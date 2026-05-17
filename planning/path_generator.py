#!/usr/bin/env python3
"""
Path Generation Module for Kinmatech Robotics
Generates joint-angle trajectories from taught program waypoints for
path validation and welding instruction planning.

Output per point:  J1..J6 (deg), Phase (STOP/ACCEL/CRUISE/DECEL),
                   X/Y/Z (mm), RX/RY/RZ (deg),
                   WeldOn (0/1), Current (%), Voltage (%),
                   WeavePattern, WeaveLat (mm), WeaveVert (mm),
                   Comment (motion type, weave info, segment label)

Design principles
─────────────────
• Joint angles include all weave displacements — they are the actual
  motor positions.  Differentiate J1-J6 to get velocity/acceleration.
• Each point carries a motion Phase (trapezoidal profile per segment):
  STOP → ACCEL → CRUISE → DECEL → STOP.  The motor controller uses
  this to switch between acceleration, constant-speed, and braking modes.
• Duplicates are aggressively removed: consecutive points whose joint delta
  is below a configurable tolerance are collapsed.
• CURVE segments require ≥ 3 waypoints (hard error, not silent fallback).
• P2P segments use stored joint angles and interpolate directly in joint space.
• Weaving overlays (sine, triangular, circular, square, zigzag, figure-8)
  are generated in Cartesian space and applied per-segment, respecting
  the weaving type set on each waypoint's weld parameters.
• Both LINEAR and CURVE motion types integrate cleanly with any weave
  pattern — the weave is computed from the interpolated path tangent/normal
  frame regardless of how the base path was generated.
"""

import sys
import os
import numpy as np
import json
from typing import List, Tuple, Dict, Any, Optional
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import splprep, splev, PchipInterpolator

# Ensure project root is in path so sibling layer modules can be found
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from planning.data_models import ProgramRow, Pose, WeldParams
from planning.kinematics_adapter import (
    KINEMATICS_SIGNATURE,
    solve_fk_gui,
    solve_ik_gui,
)


# ───────────────────────────── constants ─────────────────────────────

# Duplicate elimination: if the max joint delta between two consecutive
# trajectory points is below this threshold (degrees) the second point
# is dropped.
JOINT_DUP_TOL_DEG = 0.05

# Minimum Cartesian distance (mm) to keep a point when deduplicating
# the Cartesian interpolation *before* IK conversion.
CART_DUP_TOL_MM = 0.1

# Default interpolation spacing (mm) between points along a segment.
DEFAULT_POINT_SPACING_MM = 5.0

# Clamp: limits on interpolated points per segment (safety).
MIN_POINTS_PER_SEGMENT = 2
MAX_POINTS_PER_SEGMENT = 2000

# Joint limits (degrees) for validation.
JOINT_LIMITS = {
    0: (-170, 170), 1: (-120, 120), 2: (-70, 235),
    3: (-200, 200), 4: (-120, 120), 5: (-360, 360),
}

# Default weld values when not specified
DEFAULT_CURRENT_PCT = 0.0
DEFAULT_VOLTAGE_PCT = 0.0

# KSM metadata (must match main.py verification values)
KSM_MODEL = "KINMATECH_ROBO_ARM_1.0"
KSM_VERSION = "1.0"
KSM_FORMAT = "1"
KSM_DH_SIGNATURE = KINEMATICS_SIGNATURE

# ─────────── trapezoidal motion profile (per-segment) ────────────────
# Fraction of each segment's arc length used for accel / decel.
# The remaining middle section is CRUISE (constant speed).
# If a segment is too short for both ramps, it is split 50/50
# ACCEL + DECEL with no cruise zone.
ACCEL_FRACTION = 0.15   # first 15 % of arc → ramp up
DECEL_FRACTION = 0.15   # last  15 % of arc → ramp down

# Phase labels written to the output file
PHASE_STOP   = 'STOP'
PHASE_ACCEL  = 'ACCEL'
PHASE_CRUISE = 'CRUISE'
PHASE_DECEL  = 'DECEL'


# ────────────────────────── weave patterns ───────────────────────────

def _sine_weave(phase: float, amplitude: float) -> Tuple[float, float]:
    """Returns (lateral, vertical) offset in mm."""
    return amplitude * np.sin(phase), 0.0


def _triangular_weave(phase: float, amplitude: float) -> Tuple[float, float]:
    norm = (phase % (2 * np.pi)) / (2 * np.pi)
    if norm < 0.25:
        lat = amplitude * 4 * norm
    elif norm < 0.75:
        lat = amplitude * (2 - 4 * norm)
    else:
        lat = amplitude * (4 * norm - 4)
    return lat, 0.0


def _circular_weave(phase: float, amplitude: float) -> Tuple[float, float]:
    return amplitude * np.sin(phase), amplitude * 0.3 * np.cos(phase)


def _square_weave(phase: float, amplitude: float) -> Tuple[float, float]:
    """True square wave — full amplitude left or right with brief transitions."""
    norm = (phase % (2 * np.pi)) / (2 * np.pi)
    # Dwell at +amplitude for first half, -amplitude for second half
    # Smooth the transitions over 5% of the cycle to avoid infinite jerk
    ramp = 0.05
    if norm < ramp:
        lat = amplitude * (norm / ramp)
    elif norm < 0.5 - ramp:
        lat = amplitude
    elif norm < 0.5 + ramp:
        lat = amplitude * (1.0 - 2.0 * (norm - 0.5 + ramp) / (2 * ramp))
    elif norm < 1.0 - ramp:
        lat = -amplitude
    else:
        lat = -amplitude * (1.0 - (norm - 1.0 + ramp) / ramp)
    return lat, 0.0


def _zigzag_weave(phase: float, amplitude: float) -> Tuple[float, float]:
    """Zigzag — triangular with short dwell at extremes for better fusion."""
    norm = (phase % (2 * np.pi)) / (2 * np.pi)
    dwell = 0.08  # 8% dwell at each extreme
    if norm < dwell:
        lat = amplitude
    elif norm < 0.25:
        lat = amplitude * (1.0 - 4.0 * (norm - dwell) / (1.0 - 4 * dwell))
    elif norm < 0.5 - dwell:
        t = (norm - 0.25) / (0.25 - dwell)
        lat = -amplitude * t
    elif norm < 0.5 + dwell:
        lat = -amplitude
    elif norm < 0.75:
        lat = -amplitude * (1.0 - 4.0 * (norm - 0.5 - dwell) / (1.0 - 4 * dwell))
    elif norm < 1.0 - dwell:
        t = (norm - 0.75) / (0.25 - dwell)
        lat = amplitude * t
    else:
        lat = amplitude
    return lat, 0.0


def _figure8_weave(phase: float, amplitude: float) -> Tuple[float, float]:
    return amplitude * np.sin(phase), amplitude * 0.5 * np.sin(2 * phase)


# Map GUI weaving_type strings (case-insensitive) to pattern functions.
# 'Linear' means NO weave — it is explicitly excluded in the apply logic.
WEAVE_PATTERNS = {
    'Sine':       _sine_weave,
    'sine':       _sine_weave,
    'Triangular': _triangular_weave,
    'triangular': _triangular_weave,
    'Circle':     _circular_weave,
    'circle':     _circular_weave,
    'Zigzag':     _zigzag_weave,
    'zigzag':     _zigzag_weave,
    'Square':     _square_weave,
    'square':     _square_weave,
    'figure8':    _figure8_weave,
    'Figure8':    _figure8_weave,
}


# ═══════════════════════════ PathGenerator ════════════════════════════

class PathGenerator:
    """Generate servo-ready joint trajectories from program files."""

    def __init__(self):
        self.point_spacing_mm = DEFAULT_POINT_SPACING_MM
        self.last_generation_warnings: List[str] = []

        # Weaving defaults (can be overridden per-waypoint via weld params)
        self.weave_amplitude_mm = 3.0   # lateral amplitude
        self.weave_frequency_hz = 2.5   # oscillations per second
        self.weave_travel_speed = 50.0  # mm/s along path (for freq→wavelength)

    # ──────────────────── public entry point ─────────────────────────

    def generate_path_from_program(self, program_file_path: str) -> bool:
        """
        Load a saved program JSON, interpolate, convert to joint angles,
        eliminate duplicates, and write a clean trajectory file.
        Returns True on success.
        """
        try:
            self.last_generation_warnings = []
            program_data = self._load_program(program_file_path)
            if program_data is None:
                return False

            rows = program_data.get('rows', [])
            waypoints = self._extract_waypoints(rows)

            if len(waypoints) < 2:
                print("Error: need at least 2 motion waypoints for path generation")
                return False

            # ── 0. Fix J2/J3 configuration flips across waypoints ────
            waypoints = self._fix_waypoint_flips(waypoints)

            # ── 1. Group consecutive waypoints by motion type ────────
            segments = self._group_by_motion_type(waypoints)

            # ── 2. Validate segments ─────────────────────────────────
            for seg in segments:
                if seg['type'] == 'CURVE' and len(seg['points']) < 3:
                    print(f"Error: CURVE segment requires >= 3 points, "
                          f"got {len(seg['points'])}. "
                          f"Change motion type to LINEAR or add more waypoints.")
                    return False
                if seg['type'] == 'CURVE':
                    warning = self._curve_geometry_warning(seg['points'])
                    if warning:
                        self.last_generation_warnings.append(warning)
                        print(f"Warning: {warning}")

            # ── 3. Interpolate each segment in Cartesian space ───────
            cart_points: List[Dict] = []
            for seg in segments:
                pts = seg['points']
                if seg['type'] == 'CURVE':
                    interp = self._interpolate_curve(pts)
                elif seg['type'] == 'P2P':
                    interp = self._interpolate_p2p(pts)
                else:
                    interp = self._interpolate_linear(pts)
                cart_points.extend(interp)

            # ── 4. Remove Cartesian duplicates ───────────────────────
            cart_points = self._dedup_cartesian(cart_points)

            # ── 5. Apply weaving overlay where weld is active ────────
            cart_points = self._apply_weaving(cart_points)

            # ── 6. Remove Cartesian duplicates again (post-weave) ────
            cart_points = self._dedup_cartesian(cart_points)

            # ── 7. Convert to joint angles via IK ────────────────────
            joint_traj = self._convert_to_joints(cart_points, waypoints)

            # ── 8. Remove joint-space duplicates ─────────────────────
            joint_traj = self._dedup_joints(joint_traj)
            # ── 9. Validate joint limits without mutating the path ───
            joint_traj, limit_warnings = self._validate_joint_limits(joint_traj)
            self.last_generation_warnings.extend(limit_warnings)

            # ── 10. Assign motion phases (STOP/ACCEL/CRUISE/DECEL) ───
            joint_traj = self._assign_motion_phases(joint_traj)

            # ── 11. Write output ─────────────────────────────────────
            out_file = self.get_output_filename(program_file_path)
            self._save_trajectory(joint_traj, out_file)

            n_wp = len(waypoints)
            n_out = len(joint_traj)
            print(f"Path generation OK: {n_wp} waypoints -> {n_out} trajectory points")
            print(f"Saved to: {out_file}")
            return True

        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Path generation failed: {e}")
            return False

    # ──────────────────── file I/O helpers ───────────────────────────

    @staticmethod
    def _load_program(path: str) -> Optional[Dict]:
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading program: {e}")
            return None

    @staticmethod
    def get_output_filename(input_file: str) -> str:
        base = os.path.splitext(os.path.basename(input_file))[0]
        return os.path.join(os.path.dirname(input_file),
                            f"{base}_joint_angles.ksm")

    # ──────────────────── waypoint extraction ────────────────────────

    @staticmethod
    def _extract_waypoints(rows: List[Dict]) -> List[Dict]:
        """Pull motion waypoints out of the program rows.
        Skips TIMER / TRIGGER rows that carry no pose."""
        waypoints = []
        for row in rows:
            if row.get('pose') is None or row.get('joints_deg') is None:
                continue
            if row.get('type') in ('TIMER', 'TRIGGER'):
                continue
            joints = np.array(row['joints_deg'], dtype=float)
            pose = row['pose']
            pos = np.array([pose['x'], pose['y'], pose['z']], dtype=float)
            euler = np.array([pose['rx'], pose['ry'], pose['rz']], dtype=float)
            if row.get('type') == 'P2P':
                try:
                    fk_pose = solve_fk_gui(joints)
                    pos = np.array(fk_pose[:3], dtype=float)
                    euler = np.array(fk_pose[3:], dtype=float)
                except Exception:
                    pass
            wp = {
                'type':  row['type'],
                'pos':   pos,
                'euler': euler,
                'joints': joints,
                'speed': float(row.get('speed', 50.0)),
                'blend': float(row.get('blend', 0.0)),
                'weld':  row.get('weld', {}),
            }
            # Pre-compute quaternion for SLERP  (engine uses ZYX convention)
            q = R.from_euler(
                'ZYX',
                [wp['euler'][2], wp['euler'][1], wp['euler'][0]],
                degrees=True
            ).as_quat()
            # Enforce consistent quaternion hemisphere so SLERP always
            # takes the short arc (<180°) between consecutive waypoints.
            if waypoints and np.dot(waypoints[-1]['quat'], q) < 0:
                q = -q
            wp['quat'] = q
            waypoints.append(wp)
        return waypoints

    # ──────────────────── waypoint flip correction ───────────────────

    @staticmethod
    def _fix_waypoint_flips(waypoints: List[Dict]) -> List[Dict]:
        """Correct J2/J3 IK-branch flips across consecutive waypoints.

        Walks the waypoint list and, for each waypoint whose J2 or J3
        differs from the previous by more than a threshold, attempts to
        re-solve IK using the previous waypoint's joints as seed.  If a
        valid solution in the same branch exists, it replaces the stored
        joints so that subsequent interpolation / IK seeding stays in the
        correct arm configuration.

        This is a best-effort pre-pass — it cannot fix cases where the
        target pose genuinely requires a different IK branch.
        """
        J2J3_THRESHOLD = 20.0
        FK_TOL_MM = 1.0
        fixed = 0

        for i in range(1, len(waypoints)):
            prev_j = np.asarray(waypoints[i - 1]['joints'], dtype=float)
            cur_j = np.asarray(waypoints[i]['joints'], dtype=float)

            dj2 = abs(((cur_j[1] - prev_j[1] + 180.0) % 360.0) - 180.0)
            dj3 = abs(((cur_j[2] - prev_j[2] + 180.0) % 360.0) - 180.0)

            if dj2 < J2J3_THRESHOLD and dj3 < J2J3_THRESHOLD:
                continue

            # Build candidate seeds
            prev_norm = prev_j.copy()
            prev_norm[3:] = ((prev_norm[3:] + 180.0) % 360.0) - 180.0
            generic = [prev_j[0], 40.0, 50.0, 0.0, 0.0, 0.0]
            seeds = [prev_j.tolist(), prev_norm.tolist(), generic]

            rx, ry, rz = waypoints[i]['euler']
            pos = waypoints[i]['pos']
            pose = (pos[0], pos[1], pos[2], rx, ry, rz)

            best = cur_j
            best_cost = dj2 + dj3

            for seed in seeds:
                try:
                    alt = solve_ik_gui(pose, seed=seed)
                    if alt is None:
                        continue
                    alt = np.asarray(alt, dtype=float)

                    fk = solve_fk_gui(alt)
                    err = np.linalg.norm(
                        np.array(fk[:3]) - np.array(pos, dtype=float))
                    if err > FK_TOL_MM:
                        continue

                    new_dj2 = abs(((alt[1] - prev_j[1] + 180.0) % 360.0) - 180.0)
                    new_dj3 = abs(((alt[2] - prev_j[2] + 180.0) % 360.0) - 180.0)
                    cost = new_dj2 + new_dj3
                    if cost < best_cost:
                        best = alt
                        best_cost = cost
                except Exception:
                    continue

            if best is not cur_j:
                new_dj2 = abs(((best[1] - prev_j[1] + 180.0) % 360.0) - 180.0)
                new_dj3 = abs(((best[2] - prev_j[2] + 180.0) % 360.0) - 180.0)
                print(f"  Waypoint {i} flip fix: "
                      f"J2 {cur_j[1]:+.1f}→{best[1]:+.1f}, "
                      f"J3 {cur_j[2]:+.1f}→{best[2]:+.1f}  "
                      f"(dJ2 {dj2:.1f}→{new_dj2:.1f}, "
                      f"dJ3 {dj3:.1f}→{new_dj3:.1f})")
                waypoints[i] = dict(waypoints[i])
                waypoints[i]['joints'] = best.tolist()
                fixed += 1

        if fixed:
            print(f"Waypoint flip correction: fixed {fixed} waypoint(s)")
        return waypoints

    # ──────────────────── segment grouping ───────────────────────────

    @staticmethod
    def _same_waypoint_pose(a: Dict, b: Dict,
                            pos_tol_mm: float = CART_DUP_TOL_MM,
                            ori_tol_deg: float = 1e-3) -> bool:
        """Return True when two waypoints represent the same taught pose."""
        pa = np.asarray(a['pos'], dtype=float)
        pb = np.asarray(b['pos'], dtype=float)
        ea = np.asarray(a['euler'], dtype=float)
        eb = np.asarray(b['euler'], dtype=float)
        return (
            np.linalg.norm(pa - pb) <= float(pos_tol_mm)
            and np.max(np.abs(ea - eb)) <= float(ori_tol_deg)
        )

    @staticmethod
    def _group_by_motion_type(waypoints: List[Dict]) -> List[Dict]:
        """Group consecutive waypoints that share a motion type.
        Each group carries the *last point of the previous group* as its
        first element so that segments are continuous."""
        if not waypoints:
            return []
        segments: List[Dict] = []
        cur = {'type': waypoints[0]['type'], 'points': [waypoints[0]]}

        for wp in waypoints[1:]:
            if wp['type'] == cur['type']:
                cur['points'].append(wp)
            else:
                segments.append(cur)
                # Carry the last point forward for continuity
                carry = cur['points'][-1]
                if PathGenerator._same_waypoint_pose(carry, wp):
                    cur = {'type': wp['type'], 'points': [wp]}
                else:
                    cur = {'type': wp['type'], 'points': [carry, wp]}
        segments.append(cur)
        return segments

    # ──────────────────── interpolation ───────────────────────────────

    def _num_interp_points(self, distance_mm: float) -> int:
        n = int(np.ceil(distance_mm / self.point_spacing_mm))
        return max(MIN_POINTS_PER_SEGMENT, min(MAX_POINTS_PER_SEGMENT, n))

    @staticmethod
    def _slerp_orientation(q_start, q_end, t: float):
        """Quaternion SLERP at parameter t in [0, 1] (shortest arc)."""
        qs = np.asarray(q_start, dtype=float)
        qe = np.asarray(q_end, dtype=float)
        # Flip to ensure short-path interpolation (<180°)
        if np.dot(qs, qe) < 0:
            qe = -qe
        rots = R.from_quat(np.array([qs, qe]))
        slerp = Slerp([0.0, 1.0], rots)
        return slerp(t).as_quat()

    def _curve_geometry_warning(self, points: List[Dict]) -> Optional[str]:
        """Warn when a CURVE segment is so shallow it will look beveled."""
        clean_points: List[Dict] = []
        for pt in points:
            if not clean_points or not self._same_waypoint_pose(clean_points[-1], pt):
                clean_points.append(pt)

        if len(clean_points) < 3:
            return None

        positions = np.array([p['pos'] for p in clean_points], dtype=float)
        start = positions[0]
        end = positions[-1]
        chord = end - start
        chord_len = float(np.linalg.norm(chord))
        if chord_len <= 1e-9:
            return "Curve segment has coincident start/end points; geometry is degenerate."

        max_dev = 0.0
        for pos in positions[1:-1]:
            dev = float(np.linalg.norm(np.cross(chord, pos - start)) / chord_len)
            max_dev = max(max_dev, dev)

        warn_thresh = max(2.0 * float(self.point_spacing_mm), 0.03 * chord_len)
        if max_dev < warn_thresh:
            return (
                f"CURVE segment is shallow: max control-point deviation {max_dev:.2f} mm "
                f"over a {chord_len:.2f} mm chord. It may look like a chamfer."
            )
        return None

    def _make_point(self, pos, quat, weld, comment='',
                   speed=50.0, nominal_pos=None,
                   weave_lateral=0.0, weave_vertical=0.0,
                   weave_phase=0.0, weave_pattern='') -> Dict:
        euler_zyx = R.from_quat(quat).as_euler('ZYX', degrees=True)
        nominal = pos if nominal_pos is None else nominal_pos
        return {
            'pos':   np.array(pos, dtype=float),
            'nominal_pos': np.array(nominal, dtype=float),
            # Store as (rx, ry, rz) — GUI convention
            'euler': np.array([euler_zyx[2], euler_zyx[1], euler_zyx[0]]),
            'quat':  np.array(quat, dtype=float),
            'weld':  weld,
            'speed': float(speed),
            'comment': comment,
            # Weave displacement tracking
            'weave_lateral':  float(weave_lateral),   # mm, perpendicular to path
            'weave_vertical': float(weave_vertical),  # mm, binormal direction
            'weave_phase':    float(weave_phase),      # radians in weave cycle
            'weave_pattern':  weave_pattern,           # pattern name or ''
        }

    def _interpolate_linear(self, points: List[Dict]) -> List[Dict]:
        """Linear interpolation between consecutive waypoints.

        Position:    straight-line LERP
        Orientation: quaternion SLERP (shortest-path rotation)

        Spacing is adaptive but remains dense enough for smooth playback:
        • Base linear spacing comes from ``self.point_spacing_mm``.
        • Orientation-only moves get extra samples so wrist rotation
          does not jump between sparse checkpoints.
        • Weaving segments get denser sampling derived from the weave
          wavelength so the pattern is properly resolved.
        """
        BASE_SPACING_MM = max(float(self.point_spacing_mm), 0.5)
        POINTS_PER_CYCLE  = 10            # weave: ≥10 samples per oscillation
        ORI_STEP_DEG = 3.0                # keep wrist rotation visually smooth

        result: List[Dict] = []
        for i in range(len(points) - 1):
            A, B = points[i], points[i + 1]
            dist = np.linalg.norm(B['pos'] - A['pos'])

            # ── Decide spacing: does this sub-segment have active weaving? ──
            spacing = BASE_SPACING_MM
            for wp in (A, B):
                w = wp.get('weld', {})
                wtype = w.get('weaving_type', 'Linear')
                weld_on = w.get('on', False)
                if weld_on and wtype not in ('Linear', 'linear', '', None):
                    freq  = w.get('weave_frequency', self.weave_frequency_hz)
                    speed = max(self.weave_travel_speed, 1.0)
                    wavelength = speed / max(freq, 0.1)      # mm per cycle
                    weave_spacing = wavelength / POINTS_PER_CYCLE
                    spacing = min(spacing, weave_spacing)

            # Shortest-path quaternion angle for orientation sampling.
            q0 = np.asarray(A['quat'], dtype=float)
            q1 = np.asarray(B['quat'], dtype=float)
            qdot = float(np.clip(np.abs(np.dot(q0, q1)), 0.0, 1.0))
            ori_delta_deg = float(np.degrees(2.0 * np.arccos(qdot)))

            pos_points = self._num_interp_points(dist)
            ori_points = max(2, int(np.ceil(ori_delta_deg / ORI_STEP_DEG)) + 1)
            n = max(pos_points, ori_points)
            # Include endpoint only for the last sub-segment
            ts = np.linspace(0.0, 1.0, n,
                             endpoint=(i == len(points) - 2))

            for t in ts:
                pos = A['pos'] + t * (B['pos'] - A['pos'])
                quat = self._slerp_orientation(A['quat'], B['quat'], t)
                weld = self._lerp_weld(A['weld'], B['weld'], t)
                speed = A.get('speed', 50.0) * (1 - t) + B.get('speed', 50.0) * t
                pt = self._make_point(pos, quat, weld,
                                      f"Lin {i}-{i+1} t={t:.3f}",
                                      speed=speed)
                result.append(pt)
        return result

    @staticmethod
    def _resolve_p2p_target_joints(qa: np.ndarray, qb: np.ndarray,
                                    fk_pose_b) -> np.ndarray:
        """Re-solve IK for the P2P target using the start joints as seed.

        If the start and target joints differ significantly in J2 or J3
        (shoulder/elbow), the target may be in a flipped IK branch.  Try
        to find a solution in the same branch as the start to prevent the
        arm from sweeping through an inappropriate configuration during
        the joint-space interpolation.

        Tries multiple seed strategies:
          1. qa directly
          2. qa with wrist joints (J4-J6) normalised to [-180, 180]
          3. A generic "normal elbow" seed

        Returns *qb* unchanged when no closer valid solution exists.
        """
        J2J3_FLIP_THRESHOLD_DEG = 20.0
        FK_ROUND_TRIP_TOL_MM = 1.0

        delta_j2 = abs(((qb[1] - qa[1] + 180.0) % 360.0) - 180.0)
        delta_j3 = abs(((qb[2] - qa[2] + 180.0) % 360.0) - 180.0)

        if delta_j2 < J2J3_FLIP_THRESHOLD_DEG and delta_j3 < J2J3_FLIP_THRESHOLD_DEG:
            return qb  # no flip risk

        pose_b = (
            fk_pose_b[0], fk_pose_b[1], fk_pose_b[2],
            fk_pose_b[3], fk_pose_b[4], fk_pose_b[5],
        )

        # Build candidate seeds: qa as-is, qa with normalised wrist,
        # and a generic normal-elbow seed.
        qa_norm = qa.copy()
        qa_norm[3:] = ((qa_norm[3:] + 180.0) % 360.0) - 180.0

        generic_seed = [qa[0], 40.0, 50.0, 0.0, 0.0, 0.0]

        candidate_seeds = [qa.tolist(), qa_norm.tolist(), generic_seed]

        best = qb
        best_cost = delta_j2 + delta_j3

        for seed in candidate_seeds:
            try:
                alt = solve_ik_gui(pose_b, seed=seed)
                if alt is None:
                    continue
                alt = np.asarray(alt, dtype=float)

                # Validate FK round-trip
                fk_check = solve_fk_gui(alt)
                err = np.linalg.norm(
                    np.array(fk_check[:3]) - np.array(fk_pose_b[:3]))
                if err > FK_ROUND_TRIP_TOL_MM:
                    continue

                new_dj2 = abs(((alt[1] - qa[1] + 180.0) % 360.0) - 180.0)
                new_dj3 = abs(((alt[2] - qa[2] + 180.0) % 360.0) - 180.0)
                cost = new_dj2 + new_dj3
                if cost < best_cost:
                    best = alt
                    best_cost = cost
            except Exception:
                continue

        if best is not qb:
            orig_cost = delta_j2 + delta_j3
            new_dj2 = abs(((best[1] - qa[1] + 180.0) % 360.0) - 180.0)
            new_dj3 = abs(((best[2] - qa[2] + 180.0) % 360.0) - 180.0)
            print(f"  P2P flip guard: J2 {qb[1]:+.1f}→{best[1]:+.1f}, "
                  f"J3 {qb[2]:+.1f}→{best[2]:+.1f}  "
                  f"(dJ2 {delta_j2:.1f}→{new_dj2:.1f}, "
                  f"dJ3 {delta_j3:.1f}→{new_dj3:.1f})")

        return best

    def _interpolate_p2p(self, points: List[Dict]) -> List[Dict]:
        """Joint-space interpolation between taught waypoints.

        P2P follows the stored joint targets instead of a Cartesian path,
        then derives the displayed TCP pose from FK for each interpolated frame.
        """
        JOINT_STEP_DEG = 2.0

        result: List[Dict] = []
        for i in range(len(points) - 1):
            A, B = points[i], points[i + 1]
            qa = np.asarray(A['joints'], dtype=float)
            qb_orig = np.asarray(B['joints'], dtype=float)

            # Guard against J2/J3 configuration flips — prefer a
            # solution in the same IK branch as the departure pose.
            fk_b = solve_fk_gui(qb_orig)
            qb = self._resolve_p2p_target_joints(qa, qb_orig, fk_b)

            delta = ((qb - qa + 180.0) % 360.0) - 180.0
            max_delta = float(np.max(np.abs(delta)))
            n = max(2, int(np.ceil(max_delta / JOINT_STEP_DEG)) + 1)
            ts = np.linspace(0.0, 1.0, n, endpoint=(i == len(points) - 2))

            for t in ts:
                q = qa + t * delta
                fk_pose = solve_fk_gui(q)
                quat = R.from_euler(
                    'ZYX',
                    [fk_pose[5], fk_pose[4], fk_pose[3]],
                    degrees=True,
                ).as_quat()
                weld = self._lerp_weld(A['weld'], B['weld'], float(t))
                # P2P is joint-space motion; disable weave overlays on this segment.
                weld = dict(weld)
                weld['weaving_type'] = 'Linear'
                speed = A.get('speed', 50.0) * (1 - t) + B.get('speed', 50.0) * t
                pt = self._make_point(
                    fk_pose[:3],
                    quat,
                    weld,
                    f"P2P {i}-{i+1} t={t:.3f}",
                    speed=speed,
                    nominal_pos=fk_pose[:3],
                )
                pt['precomputed_joints'] = q.tolist()
                result.append(pt)
        return result

    def _interpolate_curve(self, points: List[Dict]) -> List[Dict]:
        """Centripetal Catmull-Rom through >= 3 waypoints + SLERP orientation.

        Uses centripetal parameterisation to avoid the overshoot / loops
        that a uniform B-spline (s=0) produces on uneven control points.
        """
        clean_points: List[Dict] = []
        for pt in points:
            pos = np.asarray(pt['pos'], dtype=float)
            if not np.all(np.isfinite(pos)):
                continue
            if clean_points:
                prev = np.asarray(clean_points[-1]['pos'], dtype=float)
                if np.linalg.norm(pos - prev) <= CART_DUP_TOL_MM:
                    continue
            clean_points.append(pt)

        if len(clean_points) < 2:
            return clean_points
        if len(clean_points) < 3:
            print("Curve interpolation fallback: fewer than 3 distinct control points")
            return self._interpolate_linear(clean_points)

        positions = np.array([p['pos'] for p in clean_points], dtype=float)
        unique_positions = np.unique(np.round(positions, decimals=6), axis=0)
        if len(unique_positions) < 3:
            print("Curve interpolation fallback: spline control points collapse to a line segment")
            return self._interpolate_linear(clean_points)

        # ── Centripetal parameterisation ──────────────────────────────
        # Chord lengths with alpha=0.5 (centripetal) prevent cusps/loops.
        dists = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        dists_sqrt = np.sqrt(dists)  # centripetal: d^0.5
        t_knots = np.concatenate([[0.0], np.cumsum(dists_sqrt)])
        t_knots /= t_knots[-1]  # normalise to [0, 1]

        # ── Per-axis PCHIP interpolation ─────────────────────────────
        # PCHIP guarantees monotonicity per segment per axis, so the
        # trajectory never overshoots beyond consecutive waypoint values.
        try:
            interp_x = PchipInterpolator(t_knots, positions[:, 0])
            interp_y = PchipInterpolator(t_knots, positions[:, 1])
            interp_z = PchipInterpolator(t_knots, positions[:, 2])
        except ValueError as exc:
            print(f"Curve interpolation fallback: {exc}")
            return self._interpolate_linear(clean_points)

        # Estimate total arc length for point count
        u_dense = np.linspace(0, 1, 500)
        dense_pts = np.column_stack([interp_x(u_dense), interp_y(u_dense), interp_z(u_dense)])
        arc = np.sum(np.linalg.norm(np.diff(dense_pts, axis=0), axis=1))
        n = self._num_interp_points(arc)

        u_new = np.linspace(0, 1, n)
        new_pos = np.column_stack([interp_x(u_new), interp_y(u_new), interp_z(u_new)])

        result: List[Dict] = []
        n_wp = len(clean_points)
        for i, (u, pos) in enumerate(zip(u_new, new_pos)):
            # Map u to segment index for orientation SLERP
            seg_i = int(np.searchsorted(t_knots[1:], u, side='right'))
            seg_i = min(seg_i, n_wp - 2)
            # Local t within this segment
            t_lo = t_knots[seg_i]
            t_hi = t_knots[seg_i + 1]
            local_t = (u - t_lo) / (t_hi - t_lo) if t_hi > t_lo else 0.0
            local_t = np.clip(local_t, 0.0, 1.0)

            A, B = clean_points[seg_i], clean_points[seg_i + 1]
            quat = self._slerp_orientation(A['quat'], B['quat'], local_t)
            weld = self._lerp_weld(A['weld'], B['weld'], local_t)
            speed = A.get('speed', 50.0) * (1 - local_t) + B.get('speed', 50.0) * local_t
            pt = self._make_point(pos, quat, weld,
                                  f"Curve u={u:.3f}",
                                  speed=speed)
            pt['seg_idx'] = seg_i
            pt['seg_local_t'] = local_t
            result.append(pt)
        return result

    # ──────────────────── weld parameter interpolation ────────────────

    @staticmethod
    def _lerp_weld(weld_a: Dict, weld_b: Dict, t: float) -> Dict:
        """Interpolate weld parameters between two waypoints.

        Boolean fields (on) snap at t=0.5.  Numeric fields (power,
        wire_feed) are linearly interpolated.  String fields (weaving_type)
        are taken from the segment that owns the majority of t.
        """
        if not weld_a and not weld_b:
            return {}
        if not weld_a:
            return dict(weld_b)
        if not weld_b:
            return dict(weld_a)

        src = weld_a if t < 0.5 else weld_b
        return {
            'on':              src.get('on', False),
            'power':           weld_a.get('power', 0) * (1 - t) + weld_b.get('power', 0) * t,
            'wire_feed':       weld_a.get('wire_feed', 0) * (1 - t) + weld_b.get('wire_feed', 0) * t,
            'gas_pre':         src.get('gas_pre', 0),
            'gas_post':        src.get('gas_post', 0),
            'weaving_type':    src.get('weaving_type', 'Linear'),
            'timer':           src.get('timer', 0),
            'sensing_trigger': src.get('sensing_trigger', 'None'),
            # Weave geometry — interpolate smoothly between waypoints
            'weave_amplitude': (weld_a.get('weave_amplitude', 3.0) * (1 - t)
                                + weld_b.get('weave_amplitude', 3.0) * t),
            'weave_frequency': (weld_a.get('weave_frequency', 2.5) * (1 - t)
                                + weld_b.get('weave_frequency', 2.5) * t),
            'weave_dwell':     (weld_a.get('weave_dwell', 0.0) * (1 - t)
                                + weld_b.get('weave_dwell', 0.0) * t),
        }

    # ──────────────────── weaving overlay ─────────────────────────────

    def _apply_weaving(self, points: List[Dict]) -> List[Dict]:
        """Overlay a lateral weave pattern on weld-active segments.

        The weaving type comes from each point's weld dict ('weaving_type').
        'Linear' means *no weave*. Anything else picks from WEAVE_PATTERNS.

        Every point in the result carries the actual weave displacement
        (weave_lateral, weave_vertical in mm) and phase so the output
        can be used to validate/visualise the weave pattern.
        """
        if len(points) < 2:
            return points

        # Pre-compute cumulative arc length for frequency -> phase mapping
        positions = np.array([p['pos'] for p in points])
        deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        cum_arc = np.concatenate([[0.0], np.cumsum(deltas)])

        # Compute path tangent and lateral (normal) vectors
        # Use per-point TCP orientation so the weave plane follows the tool
        eulers = np.array([p['euler'] for p in points])
        tangents = self._path_tangents(positions)
        normals = self._path_normals(tangents, eulers)

        result: List[Dict] = []
        for i, pt in enumerate(points):
            weld = pt.get('weld', {})
            weave_type = weld.get('weaving_type', 'Linear')
            weld_on = weld.get('on', False)

            if not weld_on or weave_type in ('Linear', 'linear', '', None):
                # No weave — pass through with zero weave fields
                result.append(pt)
                continue

            pattern_fn = WEAVE_PATTERNS.get(weave_type)
            if pattern_fn is None:
                result.append(pt)
                continue

            # Per-waypoint weave geometry (falls back to global defaults)
            amplitude = weld.get('weave_amplitude', self.weave_amplitude_mm)
            frequency = weld.get('weave_frequency', self.weave_frequency_hz)

            # Phase = 2pi * frequency * (arc / travel_speed)
            travel_speed = max(self.weave_travel_speed, 1.0)
            arc_s = cum_arc[i]
            time_s = arc_s / travel_speed  # pseudo-time along path
            phase = 2.0 * np.pi * frequency * time_s

            lat, vert = pattern_fn(phase, amplitude)

            tangent = tangents[i]
            normal = normals[i]
            binormal = np.cross(tangent, normal)

            offset = lat * normal + vert * binormal
            new_pt = pt.copy()
            new_pt['pos'] = pt['pos'] + offset
            new_pt['nominal_pos'] = np.array(pt.get('nominal_pos', pt['pos']), dtype=float)
            # Preserve original motion label, append weave tag
            base_comment = pt.get('comment', '')
            new_pt['comment'] = f"{base_comment} +{weave_type}"
            # Store actual weave displacement for output
            new_pt['weave_lateral']  = float(lat)
            new_pt['weave_vertical'] = float(vert)
            new_pt['weave_phase']    = float(phase)
            new_pt['weave_pattern']  = weave_type
            result.append(new_pt)

        return result

    # ──────────────── motion-phase assignment ─────────────────────────

    @staticmethod
    def _extract_segment_label(comment: str) -> str:
        """Extract segment identity from a trajectory comment.

        Examples
        --------
        'Lin 0-1 t=0.500'           → 'Lin 0-1'
        'Curve u=0.125 +Circle'     → 'Curve'
        'Lin 6-7 t=1.000 +Zigzag'  → 'Lin 6-7'
        """
        if not comment:
            return ''
        # Linear segments: "Lin <i>-<j> ..."
        if comment.startswith('Lin '):
            parts = comment.split()
            if len(parts) >= 2:
                return f"Lin {parts[1]}"  # e.g. "Lin 0-1"
        if comment.startswith('P2P '):
            parts = comment.split()
            if len(parts) >= 2:
                return f"P2P {parts[1]}"
        # Curve segments: "Curve ..."
        if comment.startswith('Curve'):
            return 'Curve'
        return comment.split()[0] if comment else ''

    @staticmethod
    def _assign_motion_phases(trajectory: List[Dict]) -> List[Dict]:
        """Tag every point with a trapezoidal-profile phase.

        Each segment (identified by comment prefix) gets an independent
        profile:
            ACCEL  → first ACCEL_FRACTION of the segment arc
            CRUISE → middle portion
            DECEL  → last  DECEL_FRACTION of the segment arc

        The very first and last points of the whole trajectory are
        always STOP.
        """
        if not trajectory:
            return trajectory

        # ── 1. Identify segment boundaries ───────────────────────
        # Each segment is a run of points whose comment shares the
        # same segment label.
        seg_runs: List[Tuple[int, int]] = []   # (start_idx, end_idx) inclusive
        prev_label = None
        seg_start = 0

        for i, pt in enumerate(trajectory):
            label = PathGenerator._extract_segment_label(
                pt.get('comment', ''))
            if label != prev_label:
                if prev_label is not None:
                    seg_runs.append((seg_start, i - 1))
                prev_label = label
                seg_start = i
        # close the last run
        if prev_label is not None:
            seg_runs.append((seg_start, len(trajectory) - 1))

        # ── 2. Assign phase within each segment ──────────────────
        for (si, ei) in seg_runs:
            n = ei - si + 1
            if n <= 1:
                trajectory[si]['phase'] = PHASE_STOP
                continue

            # Cumulative arc length from Cartesian X/Y/Z
            positions = np.array([
                [trajectory[si + k].get('x', 0.0),
                 trajectory[si + k].get('y', 0.0),
                 trajectory[si + k].get('z', 0.0)]
                for k in range(n)
            ])
            deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
            cum_arc = np.concatenate([[0.0], np.cumsum(deltas)])
            total_arc = cum_arc[-1]

            if total_arc < 1e-6:
                # Stationary segment — all STOP
                for k in range(n):
                    trajectory[si + k]['phase'] = PHASE_STOP
                continue

            # If segment is very short, no room for cruise:
            # split 50/50 accel + decel
            if (ACCEL_FRACTION + DECEL_FRACTION) >= 1.0:
                accel_dist = total_arc * 0.5
                decel_start = total_arc * 0.5
            else:
                accel_dist  = total_arc * ACCEL_FRACTION
                decel_start = total_arc * (1.0 - DECEL_FRACTION)

            for k in range(n):
                arc = cum_arc[k]
                if arc <= accel_dist:
                    trajectory[si + k]['phase'] = PHASE_ACCEL
                elif arc >= decel_start:
                    trajectory[si + k]['phase'] = PHASE_DECEL
                else:
                    trajectory[si + k]['phase'] = PHASE_CRUISE

        # ── 3. First & last points are always STOP ───────────────
        trajectory[0]['phase']  = PHASE_STOP
        trajectory[-1]['phase'] = PHASE_STOP

        # Summary
        counts = {}
        for pt in trajectory:
            p = pt.get('phase', '?')
            counts[p] = counts.get(p, 0) + 1
        summary = ', '.join(f"{k}: {v}" for k, v in sorted(counts.items()))
        print(f"Motion phases assigned: {summary}")

        return trajectory

    @staticmethod
    def _path_tangents(positions: np.ndarray) -> List[np.ndarray]:
        tangents = []
        n = len(positions)
        for i in range(n):
            if i == 0:
                t = positions[1] - positions[0]
            elif i == n - 1:
                t = positions[-1] - positions[-2]
            else:
                t = positions[i + 1] - positions[i - 1]
            norm = np.linalg.norm(t)
            tangents.append(t / norm if norm > 1e-9 else np.array([1, 0, 0]))
        return tangents

    @staticmethod
    def _path_normals(tangents: List[np.ndarray],
                      eulers: Optional[np.ndarray] = None) -> List[np.ndarray]:
        """Compute lateral normals for weave displacement.

        When *eulers* is provided (Nx3 array of [rx, ry, rz] per point),
        the TCP tool-axis (local Z of the end-effector) is used as the
        reference "up" direction so the weave pattern lies in the plane
        containing the tool vector and the path tangent.

        Falls back to world-Z when orientation data is unavailable.
        """
        normals = []
        for i, t in enumerate(tangents):
            # Determine reference "up" direction
            if eulers is not None and i < len(eulers):
                rx, ry, rz = eulers[i]
                tool_z = R.from_euler(
                    'ZYX', [rz, ry, rx], degrees=True
                ).as_matrix()[:, 2]   # local Z column = tool approach
                up = tool_z
            else:
                up = np.array([0.0, 0.0, 1.0])

            n = np.cross(up, t)
            nm = np.linalg.norm(n)
            if nm > 1e-9:
                normals.append(n / nm)
            else:
                alt = np.array([0.0, 1.0, 0.0])
                n = np.cross(alt, t)
                nm = np.linalg.norm(n)
                normals.append(n / nm if nm > 1e-9 else np.array([1, 0, 0]))
        return normals

    # ──────────────────── IK conversion ──────────────────────────────

    @staticmethod
    def _convert_to_joints(cart_points: List[Dict],
                           waypoints: List[Dict]) -> List[Dict]:
        """Convert trajectory points to joint space with the active backend.

        P2P points may already carry precomputed joint samples. Cartesian
        points use the active GUI-convention IK solver seeded from the
        nearest taught waypoint joints so the solver stays in the correct
        arm configuration even across large orientation changes.
        """

        result: List[Dict] = []
        ik_ok = 0

        # Build a lookup: for each trajectory point, find the closest
        # taught waypoint (by Cartesian position) and use its joints as
        # the primary IK seed.  Fall back to the previous solution.
        wp_positions = np.array([wp['pos'] for wp in waypoints], dtype=float)
        wp_joints = [np.array(wp['joints'], dtype=float).tolist() for wp in waypoints]

        prev_joints = wp_joints[0] if waypoints else None

        for idx, pt in enumerate(cart_points):
            joints = None

            if pt.get('precomputed_joints') is not None:
                joints = list(pt['precomputed_joints'])
                ik_ok += 1

            # ── 1. Try the active GUI-convention IK solver ──────────
            rx, ry, rz = pt['euler']
            if joints is None:
                # Prefer segment-aware blended seed if available
                seg_idx = pt.get('seg_idx')
                seg_t = pt.get('seg_local_t')
                if seg_idx is not None and seg_t is not None:
                    j_a = np.array(wp_joints[seg_idx], dtype=float)
                    j_b = np.array(wp_joints[min(seg_idx + 1, len(wp_joints) - 1)], dtype=float)
                    seed = ((1.0 - seg_t) * j_a + seg_t * j_b).tolist()
                else:
                    # Fallback: nearest taught waypoint by Cartesian distance
                    pt_pos = np.asarray(pt['pos'], dtype=float)
                    dists = np.linalg.norm(wp_positions - pt_pos, axis=1)
                    nearest_wp_idx = int(np.argmin(dists))
                    seed = wp_joints[nearest_wp_idx]

                # Use prev_joints (adjacent point) as primary seed for fast
                # convergence, with segment-blended seed as fallback.
                pose_tuple = (
                    pt['pos'][0], pt['pos'][1], pt['pos'][2],
                    rx, ry, rz,
                )
                seeds_to_try = [seed]
                if prev_joints is not None:
                    seeds_to_try.insert(0, prev_joints)
                for si, s in enumerate(seeds_to_try):
                    try:
                        # prev_joints probe: limit to 50 iters (adjacent point = fast)
                        # blended/nearest seed: full iterations for reliable convergence
                        mi = 50 if si == 0 and len(seeds_to_try) > 1 else None
                        ik_result = solve_ik_gui(pose_tuple, seed=s, max_iter=mi)
                        if ik_result is not None:
                            joints = np.array(ik_result, dtype=float).tolist()
                            ik_ok += 1
                            break
                    except Exception:
                        pass

            if joints is not None:
                prev_joints = joints

            if joints is None:
                x, y, z = (float(v) for v in pt['pos'])
                comment = pt.get('comment', f'point {idx + 1}')
                raise ValueError(
                    "IK failed for generated trajectory point "
                    f"{idx + 1}/{len(cart_points)} ({comment}) at "
                    f"X={x:.2f}, Y={y:.2f}, Z={z:.2f}, "
                    f"RX={float(rx):.2f}, RY={float(ry):.2f}, RZ={float(rz):.2f}. "
                    "The pose is outside the reachable workspace for the active kinematics."
                )

            weld = pt.get('weld', {})
            result.append({
                'joints':         joints,
                'x': float(pt['pos'][0]),
                'y': float(pt['pos'][1]),
                'z': float(pt['pos'][2]),
                'nominal_x': float(pt.get('nominal_pos', pt['pos'])[0]),
                'nominal_y': float(pt.get('nominal_pos', pt['pos'])[1]),
                'nominal_z': float(pt.get('nominal_pos', pt['pos'])[2]),
                'rx': float(pt['euler'][0]),
                'ry': float(pt['euler'][1]),
                'rz': float(pt['euler'][2]),
                'speed': float(pt.get('speed', 50.0)),
                'weld_on':        weld.get('on', False),
                'current_pct':    float(weld.get('power', DEFAULT_CURRENT_PCT)),
                'voltage_pct':    float(weld.get('wire_feed', DEFAULT_VOLTAGE_PCT)),
                'weave_lateral':  pt.get('weave_lateral', 0.0),
                'weave_vertical': pt.get('weave_vertical', 0.0),
                'weave_pattern':  pt.get('weave_pattern', ''),
                'comment':        pt['comment'],
            })
        print(f"IK conversion (active backend): {ik_ok} OK, 0 fallback")

        # ── Flip-correction passes ───────────────────────────────────
        # Repeatedly scan for consecutive joint jumps > threshold and
        # re-solve IK seeded from the neighbour until stable.
        FLIP_THRESH = 30.0
        MAX_PASSES = 5
        total_fixed = 0
        for pass_n in range(MAX_PASSES):
            fixed_this_pass = 0
            # Forward sweep
            for i in range(1, len(result)):
                cur = np.array(result[i]['joints'], dtype=float)
                prv = np.array(result[i - 1]['joints'], dtype=float)
                max_jump = float(np.max(np.abs(cur - prv)))
                if max_jump > FLIP_THRESH:
                    pt = cart_points[i]
                    rx, ry, rz = pt['euler']
                    pose = (pt['pos'][0], pt['pos'][1], pt['pos'][2],
                            rx, ry, rz)
                    # Try several candidate seeds
                    candidates = [prv.tolist()]
                    seg_idx = pt.get('seg_idx')
                    if seg_idx is not None:
                        candidates.append(wp_joints[seg_idx])
                        candidates.append(wp_joints[min(seg_idx + 1, len(wp_joints) - 1)])
                    best, best_jump = cur.tolist(), max_jump
                    for seed_c in candidates:
                        try:
                            ik_c = solve_ik_gui(pose, seed=seed_c, max_iter=100)
                            if ik_c is not None:
                                c = np.array(ik_c, dtype=float)
                                j = float(np.max(np.abs(c - prv)))
                                if j < best_jump:
                                    best, best_jump = c.tolist(), j
                        except Exception:
                            pass
                    if best_jump < max_jump:
                        result[i]['joints'] = best
                        # Update stored Cartesian to match corrected joints
                        try:
                            fk = solve_fk_gui(best)
                            result[i]['x'] = float(fk[0])
                            result[i]['y'] = float(fk[1])
                            result[i]['z'] = float(fk[2])
                            result[i]['rx'] = float(fk[3])
                            result[i]['ry'] = float(fk[4])
                            result[i]['rz'] = float(fk[5])
                        except Exception:
                            pass
                        fixed_this_pass += 1
            total_fixed += fixed_this_pass
            if fixed_this_pass == 0:
                break
        if total_fixed:
            print(f"Flip correction: fixed {total_fixed} point(s) in {pass_n + 1} pass(es)")
        # ── Singularity bridge ───────────────────────────────────────
        # If any consecutive joint jump > threshold remains after IK
        # re-solve attempts, bridge the gap with joint-space linear
        # interpolation over enough points to keep per-step change
        # below MAX_DEG_PER_STEP.
        MAX_DEG_PER_STEP = 10.0
        bridge_count = 0
        i = 1
        while i < len(result):
            cur = np.array(result[i]['joints'], dtype=float)
            prv = np.array(result[i - 1]['joints'], dtype=float)
            max_jump = float(np.max(np.abs(cur - prv)))
            if max_jump > FLIP_THRESH:
                # Find the span of consecutive bad points
                span_start = i
                while i < len(result):
                    c = np.array(result[i]['joints'], dtype=float)
                    p = np.array(result[i - 1]['joints'], dtype=float)
                    if float(np.max(np.abs(c - p))) <= FLIP_THRESH:
                        break
                    i += 1
                span_end = i  # first good point after the bad span
                # Calculate total joint change across the gap
                j_before = np.array(result[span_start - 1]['joints'], dtype=float)
                j_after = np.array(result[min(span_end, len(result) - 1)]['joints'], dtype=float)
                total_change = float(np.max(np.abs(j_after - j_before)))
                # Widen the bridge so each step <= MAX_DEG_PER_STEP
                needed_steps = int(np.ceil(total_change / MAX_DEG_PER_STEP))
                current_span = span_end - span_start + 1
                if needed_steps > current_span:
                    pad = (needed_steps - current_span) // 2 + 1
                    span_start = max(1, span_start - pad)
                    span_end = min(len(result), span_end + pad)
                    j_before = np.array(result[span_start - 1]['joints'], dtype=float)
                    j_after = np.array(result[min(span_end, len(result) - 1)]['joints'], dtype=float)
                n_span = span_end - span_start + 1
                for k in range(span_start, span_end):
                    t = (k - span_start + 1) / n_span
                    blended = ((1.0 - t) * j_before + t * j_after).tolist()
                    result[k]['joints'] = blended
                    # Update stored Cartesian to match FK of bridged joints
                    # so the GUI's FK consistency check passes.
                    try:
                        fk = solve_fk_gui(blended)
                        result[k]['x'] = float(fk[0])
                        result[k]['y'] = float(fk[1])
                        result[k]['z'] = float(fk[2])
                        result[k]['rx'] = float(fk[3])
                        result[k]['ry'] = float(fk[4])
                        result[k]['rz'] = float(fk[5])
                    except Exception:
                        pass
                    bridge_count += 1
            else:
                i += 1
        if bridge_count:
            print(f"Singularity bridge: interpolated {bridge_count} point(s)")

        # ── Normalize joints to hardware limits ──────────────────────
        # Revolute joints are physically identical at ±N×360° offsets.
        # The IK seed chain may wind J4/J6 beyond limits; walk forward
        # and choose the in-limit representative closest to the previous
        # point so we avoid both violations AND sudden 360° jumps.
        from planning.kin_serial import _JOINT_LIMITS
        norm_count = 0
        for idx_n in range(len(result)):
            j = result[idx_n]['joints']
            changed = False
            for ax, (lo, hi) in enumerate(_JOINT_LIMITS):
                val = j[ax]
                if lo <= val <= hi:
                    continue
                # Build candidate values: val ± N*360 that fall in [lo, hi]
                candidates = []
                for n in range(-3, 4):
                    c = val + n * 360.0
                    if lo <= c <= hi:
                        candidates.append(c)
                if not candidates:
                    continue
                # Choose the candidate closest to the previous point's value
                if idx_n > 0:
                    prev_val = result[idx_n - 1]['joints'][ax]
                    best = min(candidates, key=lambda c: abs(c - prev_val))
                else:
                    best = min(candidates, key=lambda c: abs(c - val))
                j[ax] = best
                changed = True
            if changed:
                result[idx_n]['joints'] = j
                norm_count += 1
        if norm_count:
            print(f"Joint limit normalization: shifted {norm_count} point(s)")

        return result

    # ──────────────────── joint smoothing ─────────────────────────────

    @staticmethod
    def _smooth_joints(trajectory: List[Dict], kernel: int = 5) -> List[Dict]:
        """Remove single-frame spikes and gentle jitter from joint data.

        1. **Median filter** (window=kernel) kills isolated outlier frames
           that the IK solver occasionally produces.
        2. **Moving average** (window=kernel) removes residual jitter
           without smearing the trajectory profile.

        First and last points are pinned to their original values so
        the trajectory still starts and ends at exact taught poses.
        """
        if len(trajectory) < kernel:
            return trajectory

        joints = np.array([pt['joints'] for pt in trajectory])
        n_pts, n_joints = joints.shape
        smoothed = joints.copy()

        half = kernel // 2

        # ── Pass 1: per-joint median filter ──
        for j in range(n_joints):
            col = joints[:, j]
            for i in range(half, n_pts - half):
                window = col[i - half : i + half + 1]
                smoothed[i, j] = float(np.median(window))

        # ── Pass 2: per-joint moving average ──
        avg = smoothed.copy()
        for j in range(n_joints):
            col = smoothed[:, j]
            for i in range(half, n_pts - half):
                avg[i, j] = float(np.mean(col[i - half : i + half + 1]))

        # ── Pin endpoints to exact taught values ──
        avg[0]  = joints[0]
        avg[-1] = joints[-1]

        # Write back
        for i, pt in enumerate(trajectory):
            pt['joints'] = avg[i].tolist()

        # Report magnitude of corrections
        max_correction = float(np.max(np.abs(avg - joints)))
        if max_correction > 0.1:
            print(f"Joint smoothing: max correction = {max_correction:.2f}°")

        return trajectory

    # ──────────────────── deduplication ───────────────────────────────

    @staticmethod
    def _dedup_cartesian(points: List[Dict]) -> List[Dict]:
        """Remove consecutive points whose positions are essentially identical."""
        if len(points) < 2:
            return points
        kept = [points[0]]
        for pt in points[1:]:
            dist = np.linalg.norm(pt['pos'] - kept[-1]['pos'])
            if dist >= CART_DUP_TOL_MM:
                kept.append(pt)
        # Always keep the last point
        if len(points) > 1 and not np.array_equal(kept[-1]['pos'], points[-1]['pos']):
            kept.append(points[-1])
        before = len(points)
        after = len(kept)
        if before != after:
            print(f"Cartesian dedup: {before} -> {after} "
                  f"(removed {before - after})")
        return kept

    @staticmethod
    def _dedup_joints(trajectory: List[Dict]) -> List[Dict]:
        """Remove consecutive joint-space points that are too close."""
        if len(trajectory) < 2:
            return trajectory
        kept = [trajectory[0]]
        for pt in trajectory[1:]:
            delta = np.max(np.abs(
                np.array(pt['joints']) - np.array(kept[-1]['joints'])
            ))
            if delta >= JOINT_DUP_TOL_DEG:
                kept.append(pt)
        # Always keep the final point
        if not np.array_equal(kept[-1]['joints'], trajectory[-1]['joints']):
            kept.append(trajectory[-1])
        before = len(trajectory)
        after = len(kept)
        if before != after:
            print(f"Joint dedup: {before} -> {after} "
                  f"(removed {before - after})")
        return kept

    # ──────────────────── validation ─────────────────────────────────

    @staticmethod
    def _validate_joint_limits(trajectory: List[Dict]) -> Tuple[List[Dict], List[str]]:
        """Report joint-limit violations without mutating the generated path."""
        violations = []
        for i, pt in enumerate(trajectory):
            for j in range(6):
                lo, hi = JOINT_LIMITS[j]
                val = float(pt['joints'][j])
                if val < lo or val > hi:
                    violations.append((i + 1, j + 1, val, lo, hi))

        warnings: List[str] = []
        if violations:
            first = violations[0]
            msg = (
                f"Joint limit validation: {len(violations)} value(s) outside limits; "
                f"trajectory left unchanged. First: frame {first[0]} J{first[1]}="
                f"{first[2]:.1f}° outside [{first[3]:.1f}, {first[4]:.1f}]°."
            )
            warnings.append(msg)
            print(msg)
        return trajectory, warnings

    # ──────────────────── save trajectory ─────────────────────────────

    @staticmethod
    def _save_trajectory(trajectory: List[Dict], output_file: str):
        """Write trajectory file with joint angles, Cartesian positions,
        welding instructions, and weave displacement data.

        The Cartesian X/Y/Z are the ACTUAL positions the TCP will visit
        (including weave offsets).  This lets you directly plot the
        physical trajectory and verify the weave pattern without doing
        forward kinematics.

        Format per line:
            J1..J6,  X, Y, Z, RX, RY, RZ,
            WeldOn, Current(%), Voltage(%),
            WeavePattern, WeaveLat(mm), WeaveVert(mm),
            Comment, NomX(mm), NomY(mm), NomZ(mm)
        """
        # Count summary stats for header
        n_weld = sum(1 for p in trajectory if p['weld_on'])
        n_weave = sum(1 for p in trajectory if p.get('weave_pattern', ''))
        patterns_used = sorted(set(
            p['weave_pattern'] for p in trajectory if p.get('weave_pattern', '')
        ))
        phase_counts = {}
        for p in trajectory:
            ph = p.get('phase', '?')
            phase_counts[ph] = phase_counts.get(ph, 0) + 1

        with open(output_file, 'w') as f:
            f.write("# Kinmatech State Machine Trajectory\n")
            f.write(f"# KSM_FORMAT={KSM_FORMAT}\n")
            f.write(f"# KSM_MODEL={KSM_MODEL}\n")
            f.write(f"# KSM_VERSION={KSM_VERSION}\n")
            f.write(f"# KSM_DH={KSM_DH_SIGNATURE}\n")
            f.write("# Format: J1, J2, J3, J4, J5, J6, Phase, "
                    "X(mm), Y(mm), Z(mm), RX(deg), RY(deg), RZ(deg), "
                    "WeldOn, Current(%), Voltage(%), "
                    "WeavePattern, WeaveLat(mm), WeaveVert(mm), Speed(mm/s), "
                    "Comment, NomX(mm), NomY(mm), NomZ(mm)\n")
            phase_str = '  '.join(f"{k}: {v}" for k, v in sorted(phase_counts.items()))
            f.write(f"# Total points: {len(trajectory)}  "
                    f"Weld ON: {n_weld}  "
                    f"Weave: {n_weave}")
            if patterns_used:
                f.write(f"  Patterns: {', '.join(patterns_used)}")
            f.write(f"\n# Phases: {phase_str}\n\n")

            for pt in trajectory:
                j = pt['joints']
                phase = pt.get('phase', PHASE_CRUISE)
                x  = pt.get('x', 0.0)
                y  = pt.get('y', 0.0)
                z  = pt.get('z', 0.0)
                rx = pt.get('rx', 0.0)
                ry = pt.get('ry', 0.0)
                rz = pt.get('rz', 0.0)
                weld = 1 if pt['weld_on'] else 0
                cur = pt.get('current_pct', 0.0)
                vol = pt.get('voltage_pct', 0.0)
                wpat = pt.get('weave_pattern', '')
                wlat = pt.get('weave_lateral', 0.0)
                wvert = pt.get('weave_vertical', 0.0)
                spd = pt.get('speed', 50.0)
                cmt = pt.get('comment', '')
                nx = pt.get('nominal_x', x)
                ny = pt.get('nominal_y', y)
                nz = pt.get('nominal_z', z)

                f.write(
                    f"{j[0]:.3f}, {j[1]:.3f}, {j[2]:.3f}, "
                    f"{j[3]:.3f}, {j[4]:.3f}, {j[5]:.3f}, "
                    f"{phase}, "
                    f"{x:.2f}, {y:.2f}, {z:.2f}, "
                    f"{rx:.2f}, {ry:.2f}, {rz:.2f}, "
                    f"{weld}, {cur:.1f}, {vol:.1f}, "
                    f"{wpat}, {wlat:+.2f}, {wvert:+.2f}, {spd:.2f}, "
                    f"{cmt}, {nx:.2f}, {ny:.2f}, {nz:.2f}\n"
                )

        print(f"Trajectory saved: {len(trajectory)} points -> {output_file}")
