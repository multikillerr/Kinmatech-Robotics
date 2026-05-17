#!/usr/bin/env python3
"""Offline jog knowledge preplanner.

The jog planner no longer writes one JSON file per requested jog move.
Instead it regenerates a single machine-readable KSM knowledge file around the
current robot state. The file mirrors the final generated trajectory format so
the downstream motion layer can reuse the same parsing logic.
"""

from __future__ import annotations

import os
import tempfile
from datetime import datetime
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from PyQt6.QtCore import QThread, pyqtSignal as QtSignal

from planning.kinematics_adapter import solve_fk_gui, solve_ik_gui
from planning.path_generator import (
    KSM_DH_SIGNATURE,
    KSM_FORMAT,
    KSM_MODEL,
    KSM_VERSION,
    PHASE_CRUISE,
    PHASE_STOP,
)


Pose6 = Tuple[float, float, float, float, float, float]
StopCheck = Optional[Callable[[], bool]]
AXES = ("x", "y", "z", "rx", "ry", "rz")
BUTTON_ORDER = tuple(
    (axis, direction, f"cart_{axis}{'+' if direction > 0 else '-'}")
    for axis in AXES
    for direction in (-1, 1)
)
JOG_KNOWLEDGE_SCHEMA = "kinmatech_jog_knowledge_v3"


def _should_stop(stop_check: StopCheck) -> bool:
    return bool(stop_check and stop_check())


def _to_pose6(values: Sequence[float]) -> Pose6:
    if len(values) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(values)}")
    return tuple(float(v) for v in values)


def _to_joint_list(values: Sequence[float]) -> List[float]:
    if len(values) != 6:
        raise ValueError(f"Expected 6 joint values, got {len(values)}")
    return [float(v) for v in values]


def _wrap_to_180(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def _unwrap_joints_near(candidate: Sequence[float], reference: Sequence[float]) -> List[float]:
    result: List[float] = []
    for cand, ref in zip(candidate, reference):
        value = float(cand)
        base = float(ref)
        while value - base > 180.0:
            value -= 360.0
        while value - base < -180.0:
            value += 360.0
        result.append(value)
    return result


def _pose_with_axis_delta(seed_pose: Pose6, axis: str, delta: float) -> Pose6:
    axis_map = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}
    if axis not in axis_map:
        raise ValueError(f"Unsupported jog axis: {axis}")
    pose = list(seed_pose)
    idx = axis_map[axis]
    pose[idx] = float(pose[idx]) + float(delta)
    if idx >= 3:
        pose[idx] = _wrap_to_180(pose[idx])
    return tuple(float(v) for v in pose)


def _sample_row(
    joints: Sequence[float],
    pose: Sequence[float],
    seed_joints: Sequence[float],
    seed_pose: Sequence[float],
    speed: float,
    phase: str,
    comment: str,
    button_id: str,
    axis: str,
    direction_label: str,
    step_index: int,
) -> Dict[str, object]:
    j = [float(v) for v in joints]
    p = [float(v) for v in pose]
    seed_j = [float(v) for v in seed_joints]
    seed_p = [float(v) for v in seed_pose]
    delta_pose = [
        float(p[0] - seed_p[0]),
        float(p[1] - seed_p[1]),
        float(p[2] - seed_p[2]),
        _wrap_to_180(p[3] - seed_p[3]),
        _wrap_to_180(p[4] - seed_p[4]),
        _wrap_to_180(p[5] - seed_p[5]),
    ]
    delta_joints = [float(j[i] - seed_j[i]) for i in range(6)]
    node_id = f"{button_id}:{int(step_index):03d}"
    return {
        "joints": j,
        "phase": phase,
        "x": p[0],
        "y": p[1],
        "z": p[2],
        "rx": p[3],
        "ry": p[4],
        "rz": p[5],
        "weld_on": False,
        "current_pct": 0.0,
        "voltage_pct": 0.0,
        "weave_pattern": "",
        "weave_lateral": 0.0,
        "weave_vertical": 0.0,
        "speed": float(speed),
        "comment": comment,
        "nominal_x": p[0],
        "nominal_y": p[1],
        "nominal_z": p[2],
        "kb_button": button_id,
        "kb_segment": button_id,
        "kb_axis": axis,
        "kb_direction": direction_label,
        "kb_step": int(step_index),
        "kb_node": node_id,
        "delta_pose": delta_pose,
        "delta_joints": delta_joints,
    }


def _sample_axis_direction(
    seed_joints: Sequence[float],
    seed_pose: Pose6,
    axis: str,
    direction: int,
    button_id: str,
    samples_per_direction: int,
    pos_span_mm: float,
    ori_span_deg: float,
    speed: float,
    stop_check: StopCheck = None,
) -> Dict[str, object]:
    rows: List[Dict[str, object]] = []
    sign_label = "POS" if direction > 0 else "NEG"
    span = float(pos_span_mm if axis in ("x", "y", "z") else ori_span_deg)
    step = span / float(max(1, samples_per_direction))
    previous = [float(v) for v in seed_joints]

    rows.append(
        _sample_row(
            seed_joints,
            seed_pose,
            seed_joints,
            seed_pose,
            speed,
            PHASE_STOP,
            f"KB {axis.upper()} {sign_label} seed",
            button_id,
            axis,
            sign_label,
            0,
        )
    )

    valid = 0
    last_delta = 0.0
    for sample_idx in range(1, samples_per_direction + 1):
        if _should_stop(stop_check):
            raise InterruptedError()

        delta = float(direction) * step * float(sample_idx)
        pose = _pose_with_axis_delta(seed_pose, axis, delta)
        ik = solve_ik_gui(pose)
        if ik is None:
            break

        joints = _unwrap_joints_near(ik, previous)
        phase = PHASE_STOP if sample_idx == samples_per_direction else PHASE_CRUISE
        rows.append(
            _sample_row(
                joints,
                pose,
                seed_joints,
                seed_pose,
                speed,
                phase,
                f"KB {axis.upper()} {sign_label} s={sample_idx:03d} delta={delta:+.3f}",
                button_id,
                axis,
                sign_label,
                sample_idx,
            )
        )
        previous = joints
        valid = sample_idx
        last_delta = delta

    return {
        "button": button_id,
        "segment": button_id,
        "axis": axis,
        "direction": sign_label,
        "valid_samples": int(valid),
        "reach": float(last_delta),
        "seed_node": f"{button_id}:000",
        "end_node": f"{button_id}:{int(valid):03d}",
        "rows": rows,
    }


def _infer_requested_button(request: Dict[str, object], seed_pose: Pose6) -> str:
    axis = str(request.get("axis", "") or "")
    if not axis.startswith("cart_"):
        return ""

    axis_name = axis[5:].lower()
    axis_map = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}
    idx = axis_map.get(axis_name)
    if idx is None:
        return ""

    start_pose = request.get("start_tcp")
    target_pose = request.get("target_tcp") or request.get("seed_tcp")
    try:
        if start_pose is not None and target_pose is not None:
            start = _to_pose6(start_pose)
            target = _to_pose6(target_pose)
            delta = float(target[idx] - start[idx])
            if idx >= 3:
                delta = _wrap_to_180(delta)
            if abs(delta) > 1e-9:
                return f"cart_{axis_name}{'+' if delta > 0.0 else '-'}"
    except Exception:
        pass

    reference_delta = float(seed_pose[idx])
    if idx >= 3:
        reference_delta = _wrap_to_180(reference_delta)
    if abs(reference_delta) > 1e-9:
        return f"cart_{axis_name}{'+' if reference_delta > 0.0 else '-'}"
    return ""


def _build_knowledge_blocks(
    seed_joints: Sequence[float],
    seed_pose: Pose6,
    requested_button: str,
    samples_per_direction: int,
    pos_span_mm: float,
    ori_span_deg: float,
    speed: float,
    stop_check: StopCheck = None,
) -> Tuple[List[Dict[str, object]], Dict[str, Dict[str, float]], Dict[str, Dict[str, float]]]:
    blocks: List[Dict[str, object]] = []
    summary: Dict[str, Dict[str, float]] = {}
    button_summary: Dict[str, Dict[str, float]] = {}
    ordered_buttons = list(BUTTON_ORDER)
    if requested_button:
        ordered_buttons.sort(key=lambda item: 0 if item[2] == requested_button else 1)

    for axis, direction, button_id in ordered_buttons:
        axis_summary = {
            "neg_samples": 0.0,
            "neg_reach": 0.0,
            "pos_samples": 0.0,
            "pos_reach": 0.0,
        } if axis not in summary else summary[axis]
        block = _sample_axis_direction(
            seed_joints,
            seed_pose,
            axis,
            direction,
            button_id,
            samples_per_direction,
            pos_span_mm,
            ori_span_deg,
            speed,
            stop_check=stop_check,
        )
        blocks.append(block)
        prefix = "pos" if direction > 0 else "neg"
        axis_summary[f"{prefix}_samples"] = float(block["valid_samples"])
        axis_summary[f"{prefix}_reach"] = float(block["reach"])
        summary[axis] = axis_summary
        button_summary[button_id] = {
            "axis": axis,
            "direction": float(direction),
            "samples": float(block["valid_samples"]),
            "reach": float(block["reach"]),
        }

    return blocks, summary, button_summary


def _write_knowledge_ksm(
    blocks: Sequence[Dict[str, object]],
    output_file: str,
    seed_joints: Sequence[float],
    seed_pose: Pose6,
    requested_button: str,
    axis_summary: Dict[str, Dict[str, float]],
    button_summary: Dict[str, Dict[str, float]],
    samples_per_direction: int,
    pos_span_mm: float,
    ori_span_deg: float,
) -> None:
    total_rows = sum(len(block.get("rows", [])) for block in blocks)
    tmp_file = output_file + ".tmp"
    with open(tmp_file, "w", encoding="utf-8") as fh:
        fh.write("# Kinmatech Jog Knowledge Base\n")
        fh.write(f"# KSM_FORMAT={KSM_FORMAT}\n")
        fh.write(f"# KSM_MODEL={KSM_MODEL}\n")
        fh.write(f"# KSM_VERSION={KSM_VERSION}\n")
        fh.write(f"# KSM_DH={KSM_DH_SIGNATURE}\n")
        fh.write(f"# KSM_SCHEMA={JOG_KNOWLEDGE_SCHEMA}\n")
        fh.write("# KSM_KIND=JOG_KNOWLEDGE\n")
        fh.write("# KSM_LAYOUT=BUTTON_BLOCKS_RELATIVE_TO_SEED\n")
        fh.write("# KSM_SEGMENT_KEY=JOG_BUTTON_ID\n")
        fh.write(
            "# Format: J1, J2, J3, J4, J5, J6, Phase, "
            "X(mm), Y(mm), Z(mm), RX(deg), RY(deg), RZ(deg), "
            "WeldOn, Current(%), Voltage(%), "
            "WeavePattern, WeaveLat(mm), WeaveVert(mm), Speed(mm/s), "
            "Comment, NomX(mm), NomY(mm), NomZ(mm), "
            "KBButton, KBSegment, KBAxis, KBDirection, KBStep, KBNode, "
            "DeltaX(mm), DeltaY(mm), DeltaZ(mm), DeltaRX(deg), DeltaRY(deg), DeltaRZ(deg), "
            "DeltaJ1(deg), DeltaJ2(deg), DeltaJ3(deg), DeltaJ4(deg), DeltaJ5(deg), DeltaJ6(deg)\n"
        )
        fh.write(
            "# SeedJoints="
            + ", ".join(f"{float(v):.3f}" for v in seed_joints)
            + "\n"
        )
        fh.write(
            "# SeedTCP="
            + ", ".join(f"{float(v):.3f}" for v in seed_pose)
            + "\n"
        )
        if requested_button:
            fh.write(f"# RequestedButton={requested_button}\n")
        fh.write(
            f"# SamplesPerDirection={samples_per_direction}  "
            f"PositionSpan(mm)={pos_span_mm:.3f}  "
            f"OrientationSpan(deg)={ori_span_deg:.3f}\n"
        )
        for axis in AXES:
            s = axis_summary.get(axis, {})
            fh.write(
                f"# Axis {axis.upper()}: "
                f"NEG samples={int(s.get('neg_samples', 0))} reach={float(s.get('neg_reach', 0.0)):+.3f}  "
                f"POS samples={int(s.get('pos_samples', 0))} reach={float(s.get('pos_reach', 0.0)):+.3f}\n"
            )
        for button_id, meta in button_summary.items():
            fh.write(
                f"# Segment {button_id}: axis={meta.get('axis')} "
                f"direction={int(meta.get('direction', 0)):+d} "
                f"samples={int(meta.get('samples', 0))} "
                f"reach={float(meta.get('reach', 0.0)):+.3f} "
                f"seed_node={button_id}:000 "
                f"end_node={button_id}:{int(meta.get('samples', 0)):03d}\n"
            )
        fh.write(f"# Total points: {total_rows}\n\n")

        for block in blocks:
            fh.write(
                f"# BLOCK segment={block.get('segment')} button={block.get('button')} axis={block.get('axis')} "
                f"direction={block.get('direction')} samples={int(block.get('valid_samples', 0))} "
                f"reach={float(block.get('reach', 0.0)):+.3f} "
                f"seed_node={block.get('seed_node')} end_node={block.get('end_node')}\n"
            )
            for row in block.get("rows", []):
                joints = row["joints"]
                delta_pose = row.get("delta_pose", [0.0] * 6)
                delta_joints = row.get("delta_joints", [0.0] * 6)
                fh.write(
                    f"{joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}, "
                    f"{joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}, "
                    f"{row.get('phase', PHASE_CRUISE)}, "
                    f"{float(row.get('x', 0.0)):.2f}, {float(row.get('y', 0.0)):.2f}, {float(row.get('z', 0.0)):.2f}, "
                    f"{float(row.get('rx', 0.0)):.2f}, {float(row.get('ry', 0.0)):.2f}, {float(row.get('rz', 0.0)):.2f}, "
                    f"{1 if row.get('weld_on') else 0}, "
                    f"{float(row.get('current_pct', 0.0)):.1f}, {float(row.get('voltage_pct', 0.0)):.1f}, "
                    f"{row.get('weave_pattern', '')}, "
                    f"{float(row.get('weave_lateral', 0.0)):+.2f}, {float(row.get('weave_vertical', 0.0)):+.2f}, "
                    f"{float(row.get('speed', 0.0)):.2f}, "
                    f"{row.get('comment', '')}, "
                    f"{float(row.get('nominal_x', 0.0)):.2f}, {float(row.get('nominal_y', 0.0)):.2f}, {float(row.get('nominal_z', 0.0)):.2f}, "
                    f"{row.get('kb_button', '')}, {row.get('kb_segment', '')}, {row.get('kb_axis', '')}, {row.get('kb_direction', '')}, {int(row.get('kb_step', 0))}, {row.get('kb_node', '')}, "
                    f"{float(delta_pose[0]):+.3f}, {float(delta_pose[1]):+.3f}, {float(delta_pose[2]):+.3f}, "
                    f"{float(delta_pose[3]):+.3f}, {float(delta_pose[4]):+.3f}, {float(delta_pose[5]):+.3f}, "
                    f"{float(delta_joints[0]):+.3f}, {float(delta_joints[1]):+.3f}, {float(delta_joints[2]):+.3f}, "
                    f"{float(delta_joints[3]):+.3f}, {float(delta_joints[4]):+.3f}, {float(delta_joints[5]):+.3f}\n"
                )
            fh.write("\n")

    os.replace(tmp_file, output_file)


def build_jog_preplan(
    request: Dict[str, object],
    stop_check: StopCheck = None,
) -> Dict[str, object]:
    seed_joints = _to_joint_list(
        request.get("seed_joints")
        or request.get("target_joints")
        or request.get("start_joints")
    )
    seed_pose = _to_pose6(request.get("seed_tcp") or request.get("target_tcp") or solve_fk_gui(seed_joints))
    speed = float(request.get("speed", 50.0))
    output_dir = str(
        request.get("output_dir")
        or os.path.join(tempfile.gettempdir(), "kinmatech_jog_preplans")
    )
    samples_per_direction = max(4, int(request.get("samples_per_direction", 60)))
    pos_span_mm = max(1.0, float(request.get("position_span_mm", 60.0)))
    ori_span_deg = max(1.0, float(request.get("orientation_span_deg", 30.0)))
    requested_button = _infer_requested_button(request, seed_pose)

    os.makedirs(output_dir, exist_ok=True)
    for name in os.listdir(output_dir):
        if name.startswith("active_jog_") or name.startswith("jog_"):
            path = os.path.join(output_dir, name)
            if os.path.isfile(path):
                try:
                    os.remove(path)
                except OSError:
                    pass

    blocks, axis_summary, button_summary = _build_knowledge_blocks(
        seed_joints,
        seed_pose,
        requested_button,
        samples_per_direction,
        pos_span_mm,
        ori_span_deg,
        speed,
        stop_check=stop_check,
    )
    if _should_stop(stop_check):
        raise InterruptedError()

    file_path = os.path.join(output_dir, "active_jog_knowledge.ksm")
    _write_knowledge_ksm(
        blocks,
        file_path,
        seed_joints,
        seed_pose,
        requested_button,
        axis_summary,
        button_summary,
        samples_per_direction,
        pos_span_mm,
        ori_span_deg,
    )

    return {
        "schema": JOG_KNOWLEDGE_SCHEMA,
        "request_id": int(request.get("request_id", 0)),
        "generated_at": datetime.now().isoformat(timespec="milliseconds"),
        "motion_source": str(request.get("motion_source", "manual")),
        "axis": str(request.get("axis", "unknown")),
        "requested_button": requested_button,
        "requested_segment": requested_button,
        "seed_joints": seed_joints,
        "seed_tcp": list(seed_pose),
        "samples_per_direction": samples_per_direction,
        "position_span_mm": pos_span_mm,
        "orientation_span_deg": ori_span_deg,
        "frame_count": sum(len(block.get("rows", [])) for block in blocks),
        "axis_summary": axis_summary,
        "button_summary": button_summary,
        "segment_summary": button_summary,
        "file_path": file_path,
    }


class JogPreplannerThread(QThread):
    """Build a replaceable jog knowledge file off the UI thread."""

    plan_ready = QtSignal(dict)
    log_message = QtSignal(str, str)
    error = QtSignal(str)

    def __init__(self, request: Dict[str, object], parent=None):
        super().__init__(parent)
        self.request = dict(request)

    def run(self) -> None:
        request_id = int(self.request.get("request_id", 0))
        axis = str(self.request.get("axis", "unknown"))
        self.log_message.emit(
            f"Jog knowledge build started: {axis} request={request_id}",
            "DEBUG",
        )
        try:
            plan = build_jog_preplan(
                self.request,
                stop_check=self.isInterruptionRequested,
            )
        except InterruptedError:
            self.log_message.emit(
                f"Jog knowledge build cancelled: {axis} request={request_id}",
                "DEBUG",
            )
            return
        except Exception as exc:
            self.error.emit(f"Jog knowledge build failed: {exc}")
            return

        if self.isInterruptionRequested():
            self.log_message.emit(
                f"Jog knowledge build discarded after cancellation: {axis} request={request_id}",
                "DEBUG",
            )
            return

        self.plan_ready.emit(plan)
