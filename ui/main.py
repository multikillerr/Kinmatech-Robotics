#!/usr/bin/env python3
"""
Kinmatech Robotics Main Application
PyQt6 application with 3-column + bottom layout
"""

import sys
import time
import json
import os
import glob
from datetime import datetime, timedelta
from typing import Optional, List
import numpy as np

# Ensure project root is in path so sibling layer modules can be found
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R_scipy, Slerp
from planning.kinematics_adapter import (
    KINEMATICS_SIGNATURE,
    solve_fk_gui,
    solve_ik_gui,
    solve_visual_chain_gui,
)
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGroupBox, QTableView, QToolBar, 
                             QStatusBar, QPushButton, QLabel, QGridLayout,
                             QSlider, QTabWidget, QLineEdit, QComboBox, QSpinBox,
                             QFileDialog, QInputDialog, QMessageBox, QTextEdit)
from PyQt6.QtCore import Qt, QTimer, pyqtSlot, pyqtSignal as QtSignal, QMutex, QMutexLocker, QThread
from PyQt6.QtGui import QAction, QFont

# Import threading components from control layer
from control.threads import ControlThread, HardwareThread
from control.command_queue import ThreadSafeCommandQueue
from control.jog_preplanner import JogPreplannerThread
from hardware.pendant_class import PendantConnectionThread

# Import data models for future integration
from planning.data_models import Pose, WeldParams, ProgramRow, RobotState, ProgramManager
from ui.program_table_model import ProgramTableModel
from control.kin_worker import KinematicThread
from planning.path_generator import PathGenerator


# KSM (Kinmatech State Machine) metadata for machine-bound trajectories
KSM_MODEL = "KINMATECH_ROBO_ARM_1.0"
KSM_VERSION = "1.0"
KSM_FORMAT = "1"
KSM_DH_SIGNATURE = KINEMATICS_SIGNATURE


def read_ksm_metadata(file_path: str) -> dict:
    """Read # KEY=VALUE KSM headers from a .ksm file."""
    meta = {}
    if not file_path.lower().endswith('.ksm'):
        return meta
    try:
        with open(file_path, 'r') as fh:
            for raw in fh:
                line = raw.strip()
                if not line:
                    continue
                if not line.startswith('#'):
                    break
                body = line[1:].strip()
                if '=' in body:
                    k, v = body.split('=', 1)
                    meta[k.strip()] = v.strip()
    except Exception:
        return {}
    return meta


def validate_ksm_metadata(meta: dict) -> tuple:
    """Validate that trajectory metadata matches this hardcoded machine."""
    if not meta:
        return False, "Missing KSM metadata"

    checks = {
        'KSM_MODEL': KSM_MODEL,
        'KSM_VERSION': KSM_VERSION,
        'KSM_DH': KSM_DH_SIGNATURE,
        'KSM_FORMAT': KSM_FORMAT,
    }
    for key, expected in checks.items():
        actual = meta.get(key)
        if actual is None:
            return False, f"Missing header: {key}"
        if actual != expected:
            return False, f"{key} mismatch (file={actual}, machine={expected})"
    return True, "OK"


def create_empty_program_rows() -> List[ProgramRow]:
    """Create empty program rows list"""
    return []


class Robot3DVisualizer(QWidget):
    """PyQtGraph OpenGL robot visualiser with coalesced real-time updates."""

    frame_changed = QtSignal(int, list)
    playback_finished = QtSignal()

    _LINK_COLORS = [
        (0.91, 0.30, 0.24, 1.0),
        (0.90, 0.49, 0.13, 1.0),
        (0.95, 0.77, 0.06, 1.0),
        (0.18, 0.80, 0.44, 1.0),
        (0.20, 0.60, 0.86, 1.0),
        (0.61, 0.35, 0.71, 1.0),
    ]

    def __init__(self):
        super().__init__()
        self.mutex = QMutex()

        self.current_joints = [0.0] * 6
        self.tcp_positions = []
        self._tcp_segment_lengths = []
        self._tcp_path_total_m = 0.0
        self.cleanup_in_progress = False
        self._prev_orient = None
        self._last_info_text = ""
        self._last_status_text = ""
        self._empty_line_pos = np.zeros((1, 3), dtype=float)

        self.traj_joints = None
        self.traj_cartesian = []
        self.traj_has_cartesian = False
        self.traj_playing = False
        self.traj_paused = False
        self.traj_frame = 0
        self.traj_woven_trail = []
        self.traj_weld_flags = []
        self.traj_orientation_deltas = []
        self.traj_timer = QTimer()
        self.traj_timer.timeout.connect(self._traj_tick)
        self.traj_interval_ms = 30
        self.traj_speed_mult = 1.0
        self.traj_min_interval_ms = 8
        self.traj_max_interval_ms = 40
        # Frame skipping throttle: skip UI updates if playback too fast
        self._traj_frame_display_skip = 0
        self._traj_frame_display_threshold = 2  # Update display every N frames
        self.render_timer = QTimer()
        self.render_timer.setSingleShot(True)
        self.render_timer.timeout.connect(self.update_robot_display)
        # Increase rendering interval slightly during playback to reduce GPU redraw
        self.render_min_interval_ms = 20

        self._link_items = []
        self._joint_items = []
        self._base_item = None
        self._tcp_item = None
        self._tcp_mount_item = None
        self._tcp_axis_items = []
        self._tcp_path_item = None
        self._recent_path_item = None
        self._woven_path_item = None
        self._recent_woven_path_item = None

        self.setup_ui()

    def setup_ui(self):
        pg.setConfigOptions(antialias=True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor('#1e1e2e')
        self.view.setMinimumHeight(320)
        self.reset_view()
        layout.addWidget(self.view, 1)

        grid = gl.GLGridItem()
        grid.setSize(x=2.5, y=2.5)
        grid.setSpacing(x=0.25, y=0.25)
        self.view.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(0.4, 0.4, 0.4)
        self.view.addItem(axis)

        # Visual XYZ markers for easier spatial analysis.
        x_marker = gl.GLScatterPlotItem(pos=np.array([[0.45, 0.0, 0.0]]), color=(1.0, 0.25, 0.25, 1.0), size=8)
        y_marker = gl.GLScatterPlotItem(pos=np.array([[0.0, 0.45, 0.0]]), color=(0.25, 1.0, 0.25, 1.0), size=8)
        z_marker = gl.GLScatterPlotItem(pos=np.array([[0.0, 0.0, 0.45]]), color=(0.30, 0.65, 1.0, 1.0), size=8)
        self.view.addItem(x_marker)
        self.view.addItem(y_marker)
        self.view.addItem(z_marker)

        for color in self._LINK_COLORS:
            link_item = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=color, width=3, antialias=True)
            self.view.addItem(link_item)
            self._link_items.append(link_item)

            joint_item = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=color, size=10)
            self.view.addItem(joint_item)
            self._joint_items.append(joint_item)

        self._base_item = gl.GLScatterPlotItem(pos=np.array([[0.0, 0.0, 0.0]]), color=(1.0, 1.0, 1.0, 1.0), size=14)
        self.view.addItem(self._base_item)

        self._tcp_item = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=(1.0, 0.92, 0.23, 1.0), size=14)
        self.view.addItem(self._tcp_item)

        self._tcp_mount_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3)),
            color=(0.95, 0.95, 0.95, 0.9),
            width=2,
            antialias=True,
        )
        self.view.addItem(self._tcp_mount_item)

        for color in (
            (1.0, 0.25, 0.25, 0.95),
            (0.25, 1.0, 0.25, 0.95),
            (0.30, 0.65, 1.0, 0.95),
        ):
            axis_item = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=color, width=2, antialias=True)
            self.view.addItem(axis_item)
            self._tcp_axis_items.append(axis_item)

        self._tcp_path_item = gl.GLLinePlotItem(pos=np.zeros((1, 3)), color=(0.40, 0.73, 0.42, 0.55), width=2, antialias=True)
        self._recent_path_item = gl.GLLinePlotItem(pos=np.zeros((1, 3)), color=(0.30, 0.69, 0.31, 0.95), width=3, antialias=True)
        self._woven_path_item = gl.GLLinePlotItem(pos=np.zeros((1, 3)), color=(1.0, 0.60, 0.0, 0.85), width=3, antialias=True)
        self._recent_woven_path_item = gl.GLLinePlotItem(pos=np.zeros((1, 3)), color=(1.0, 0.34, 0.13, 0.95), width=4, antialias=True)
        for item in (
            self._tcp_path_item,
            self._recent_path_item,
            self._woven_path_item,
            self._recent_woven_path_item,
        ):
            self.view.addItem(item)

        ctrl = QHBoxLayout()
        ctrl.setSpacing(4)
        btn_css = "QPushButton { padding: 4px 10px; font-weight: bold; }"

        self.play_btn = QPushButton("▶ Play")
        self.play_btn.setStyleSheet(btn_css)
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self.toggle_playback)
        self.play_btn.hide()

        self.stop_btn = QPushButton("⏹ Stop")
        self.stop_btn.setStyleSheet(btn_css)
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_playback)
        ctrl.addWidget(self.stop_btn)

        ctrl.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 300)
        self.speed_slider.setValue(100)
        self.speed_slider.setMaximumWidth(120)
        self.speed_slider.valueChanged.connect(self._on_speed_slider)
        ctrl.addWidget(self.speed_slider)
        self.speed_label = QLabel("1.0×")
        ctrl.addWidget(self.speed_label)

        ctrl.addStretch()

        self.clear_path_btn = QPushButton("Clear Path")
        self.clear_path_btn.clicked.connect(self.clear_tcp_path)
        ctrl.addWidget(self.clear_path_btn)

        self.reset_view_btn = QPushButton("Reset View")
        self.reset_view_btn.clicked.connect(self.reset_view)
        ctrl.addWidget(self.reset_view_btn)

        layout.addLayout(ctrl)

        self.info_label = QLabel("")
        self.info_label.setStyleSheet(
            "color: #d6d6d6; background-color: #202432; padding: 6px;"
            "font-family: Menlo, Monaco, monospace; font-size: 10px;"
        )
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label)

        axis_hint = QLabel("Axis markers: X=red  Y=green  Z=blue")
        axis_hint.setStyleSheet("color: #95a5a6; font-size: 10px;")
        layout.addWidget(axis_hint)

        self.status_line = QLabel("")
        self.status_line.setStyleSheet("color: #aaa; font-size: 10px;")
        layout.addWidget(self.status_line)

        self.update_robot_display()

    # ----------------------------------- Backend-consistent display kinematics
    def forward_kinematics(self, joint_angles):
        """Compute display positions from the active backend geometry."""
        return solve_visual_chain_gui(joint_angles)

    # ------------------------------------- Live joint updates (from jog/IK)
    def update_joints(self, joint_angles):
        """Thread-safe live update (ignored while trajectory is playing)."""
        if self.cleanup_in_progress or self.traj_playing:
            return
        with QMutexLocker(self.mutex):
            self.current_joints = list(joint_angles)
        self.schedule_render()

    def schedule_render(self):
        """Coalesce multiple update requests into a single canvas redraw."""
        if self.cleanup_in_progress:
            return
        if not self.render_timer.isActive():
            self.render_timer.start(self.render_min_interval_ms)

    def _set_status_text(self, text: str):
        """Avoid redundant QLabel updates in high-frequency render paths."""
        if text != self._last_status_text:
            self.status_line.setText(text)
            self._last_status_text = text

    def _set_info_text(self, text: str):
        """Avoid redundant QLabel updates in high-frequency render paths."""
        if text != self._last_info_text:
            self.info_label.setText(text)
            self._last_info_text = text

    # ======================== TRAJECTORY LOADING (from anime_2.py) =========
    def load_trajectory_file(self, file_path):
        """Parse a generated joint-angles file (same format as anime_2.py)."""
        if file_path.lower().endswith('.ksm'):
            ok, msg = validate_ksm_metadata(read_ksm_metadata(file_path))
            if not ok:
                self.status_line.setText(
                    f"KSM verify failed: {msg} — press 'Generate Path' to fix"
                )
                return False

        joint_rows = []
        cart_rows = []
        has_cart = False

        with open(file_path, 'r') as fh:
            for line in fh:
                if line.startswith('#') or '--- Timer' in line or '--- Trigger' in line:
                    continue
                parts = [x.strip() for x in line.split(',')]
                if len(parts) < 6:
                    continue
                try:
                    ja = [float(parts[k]) for k in range(6)]
                    joint_rows.append(ja)
                    if len(parts) >= 19:
                        try:
                            cd = {
                                'phase':          parts[6],
                                'position':       np.array([float(parts[7]), float(parts[8]), float(parts[9])]),
                                'orientation':    np.array([float(parts[10]), float(parts[11]), float(parts[12])]),
                                'weld_on':        int(float(parts[13])),
                                'current_pct':    float(parts[14]),
                                'voltage_pct':    float(parts[15]),
                                'weave_pattern':  parts[16],
                                'weave_lateral':  float(parts[17]),
                                'weave_vertical': float(parts[18]),
                                'segment_speed':  float(parts[19]) if len(parts) >= 21 else 0.0,
                            }
                            if len(parts) >= 24:
                                cd['nominal_position'] = np.array([
                                    float(parts[21]), float(parts[22]), float(parts[23])
                                ])
                            cart_rows.append(cd)
                            has_cart = True
                        except (ValueError, IndexError):
                            pass
                except (ValueError, IndexError):
                    continue

        if not joint_rows:
            return False

        self.traj_joints = np.array(joint_rows)
        self.traj_cartesian = cart_rows
        self.traj_has_cartesian = has_cart and len(cart_rows) == len(joint_rows)
        self.traj_frame = 0
        self.traj_woven_trail = []
        self.traj_weld_flags = []
        self.traj_orientation_deltas = []
        self.tcp_positions = []
        self._tcp_segment_lengths = []
        self._tcp_path_total_m = 0.0

        self._calc_playback_interval()

        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(True)
        info = f"Loaded {len(joint_rows)} frames"
        if has_cart:
            info += " with Cartesian + weave data"
        self._set_status_text(info)
        return True

    def _calc_playback_interval(self):
        """Choose interval to approximate 200 mm/s (same logic as anime_2)."""
        if self.traj_joints is None or len(self.traj_joints) < 2:
            self.traj_interval_ms = 30
            return
        sample = min(50, len(self.traj_joints) - 1)
        total_d = 0.0
        for i in range(1, sample + 1):
            p1, _ = self.forward_kinematics(self.traj_joints[i - 1])
            p2, _ = self.forward_kinematics(self.traj_joints[i])
            total_d += np.linalg.norm(p2[-1] - p1[-1])
        avg_mm = (total_d / sample) * 1000.0
        target_speed = 200.0  # mm/s
        if avg_mm > 0:
            ms = (avg_mm / target_speed) * 1000.0
            n = len(self.traj_joints)
            if n > 1000:
                ms = max(1, min(20, ms))
            elif n > 500:
                ms = max(2, min(50, ms))
            else:
                ms = max(5, min(100, ms))
        else:
            ms = 10 if len(self.traj_joints) > 500 else 25
        self.traj_interval_ms = int(ms)

    def _frame_interval_ms(self, frame_idx: int) -> int:
        """Return per-frame playback interval, honoring embedded trajectory speed."""
        def _clamp_interval(ms_value: float) -> int:
            ms_value = float(ms_value)
            return int(max(self.traj_min_interval_ms, min(self.traj_max_interval_ms, ms_value)))

        base = self.traj_interval_ms
        if not (self.traj_has_cartesian and 0 <= frame_idx < len(self.traj_cartesian)):
            return _clamp_interval(base / self.traj_speed_mult)

        cd = self.traj_cartesian[frame_idx]
        seg_speed = float(cd.get('segment_speed', 0.0) or 0.0)
        if seg_speed <= 0.0:
            return _clamp_interval(base / self.traj_speed_mult)

        # Use Cartesian distance between consecutive frames for true TCP speed timing.
        if frame_idx > 0:
            p0 = self.traj_cartesian[frame_idx - 1].get('position')
            p1 = cd.get('position')
        elif len(self.traj_cartesian) > 1:
            p0 = cd.get('position')
            p1 = self.traj_cartesian[frame_idx + 1].get('position')
        else:
            p0 = p1 = None

        if p0 is None or p1 is None:
            return _clamp_interval(base / self.traj_speed_mult)

        dist_mm = float(np.linalg.norm(p1 - p0))
        if dist_mm <= 1e-6:
            return _clamp_interval(base / self.traj_speed_mult)

        interval = (dist_mm / max(seg_speed, 0.1)) * 1000.0
        return _clamp_interval(interval / self.traj_speed_mult)

    # ======================== PLAYBACK CONTROLS ============================
    def toggle_playback(self):
        if self.traj_joints is None:
            return
        if self.traj_playing and not self.traj_paused:
            # Pause
            self.traj_paused = True
            self.traj_timer.stop()
            self.play_btn.setText("▶ Resume")
            self._set_status_text(f"Paused at frame {self.traj_frame}/{len(self.traj_joints)}")
        else:
            if not self.traj_paused:
                # Fresh start
                self.traj_frame = 0
                self.traj_woven_trail = []
                self.traj_orientation_deltas = []
                self.tcp_positions = []
                self._tcp_segment_lengths = []
                self._tcp_path_total_m = 0.0
            self.traj_paused = False
            self.traj_playing = True
            interval = self._frame_interval_ms(self.traj_frame)
            self.traj_timer.start(interval)
            self.play_btn.setText("⏸ Pause")

    def stop_playback(self):
        self.traj_timer.stop()
        self.traj_playing = False
        self.traj_paused = False
        self.traj_frame = 0
        self.traj_woven_trail = []
        self.traj_orientation_deltas = []
        self.tcp_positions = []
        self._tcp_segment_lengths = []
        self._tcp_path_total_m = 0.0
        self.play_btn.setText("▶ Play")
        self.play_btn.setEnabled(self.traj_joints is not None)
        self._set_status_text("Stopped")
        self.schedule_render()

    def _traj_tick(self):
        if self.traj_joints is None or self.traj_frame >= len(self.traj_joints):
            self.traj_timer.stop()
            self.traj_playing = False
            self.play_btn.setText("▶ Play")
            self._set_status_text("Playback complete")
            self._traj_frame_display_skip = 0  # Reset skip counter
            self.playback_finished.emit()
            return

        joints = self.traj_joints[self.traj_frame].tolist()
        with QMutexLocker(self.mutex):
            self.current_joints = joints
        
        # Only emit display updates every N frames to reduce UI event queue pressure
        if self._traj_frame_display_skip == 0:
            self.frame_changed.emit(self.traj_frame, joints)
        
        self._traj_frame_display_skip = (self._traj_frame_display_skip + 1) % self._traj_frame_display_threshold
        self.schedule_render()
        self.traj_frame += 1

        # Update next interval using per-point speed profile.
        if self.traj_playing and self.traj_frame < len(self.traj_joints):
            self.traj_timer.setInterval(self._frame_interval_ms(self.traj_frame))

    def _on_speed_slider(self, value):
        self.traj_speed_mult = value / 100.0
        self.speed_label.setText(f"{self.traj_speed_mult:.1f}×")
        if self.traj_playing and not self.traj_paused:
            interval = self._frame_interval_ms(self.traj_frame)
            self.traj_timer.setInterval(interval)

    def _set_line_item(self, item, points):
        if points is None or len(points) < 2:
            item.setData(pos=self._empty_line_pos)
            return
        item.setData(pos=np.asarray(points, dtype=float))

    def _set_line_item_with_colors(self, item, points, colors):
        if points is None or len(points) < 2:
            item.setData(pos=self._empty_line_pos)
            return
        pos = np.asarray(points, dtype=float)
        col = np.asarray(colors, dtype=float) if colors is not None else None
        if col is None or len(col) != len(pos):
            item.setData(pos=pos)
            return
        item.setData(pos=pos, color=col)

    def _build_weld_path_colors(self, weld_flags):
        if not weld_flags:
            return None
        # OFF: green (normal jog color), ON: yellow (active weld color)
        off = np.array([0.30, 0.69, 0.31, 0.95], dtype=float)
        on = np.array([1.0, 0.92, 0.23, 0.95], dtype=float)
        return np.array([on if bool(flag) else off for flag in weld_flags], dtype=float)

    def _set_scatter_item(self, item, point, color=None, size=None):
        kwargs = {"pos": np.asarray([point], dtype=float)}
        if color is not None:
            kwargs["color"] = color
        if size is not None:
            kwargs["size"] = size
        item.setData(**kwargs)

    def _set_tcp_axes(self, origin, orientation_deg):
        """Draw a small TCP orientation triad from XYZ Euler angles."""
        if not self._tcp_axis_items:
            return

        origin = np.asarray(origin, dtype=float)
        rot = R_scipy.from_euler("xyz", [float(v) for v in orientation_deg], degrees=True)
        basis = rot.as_matrix()
        mount_len_m = 0.06
        axis_len_m = 0.18
        mount = origin - (basis[:, 2] * mount_len_m)
        if self._tcp_mount_item is not None:
            self._set_line_item(self._tcp_mount_item, np.vstack([mount, origin]))
        for idx, item in enumerate(self._tcp_axis_items):
            tip = origin + (basis[:, idx] * axis_len_m)
            self._set_line_item(item, np.vstack([origin, tip]))

    def _update_info_panel(self, joints, tcp_pos, orientations, cd):
        lines = []
        joint_text = '  '.join(f'J{i + 1}:{value:6.1f}°' for i, value in enumerate(joints))
        lines.append(joint_text)
        lines.append(
            f'TCP  X={tcp_pos[0] * 1000:.1f}  Y={tcp_pos[1] * 1000:.1f}  Z={tcp_pos[2] * 1000:.1f} mm'
        )

        if orientations:
            rx, ry, rz = orientations[-1]
            lines.append(f'ORI  RX={rx:.1f}°  RY={ry:.1f}°  RZ={rz:.1f}°')

        if self.traj_playing and self.traj_joints is not None:
            total = len(self.traj_joints)
            pct = 100 * self.traj_frame / total if total else 0.0
            lines.append(f'PLAY Frame {self.traj_frame}/{total} ({pct:.1f}%)')
            if cd is not None:
                phase = cd.get('phase', '')
                if phase:
                    lines.append(f'PHASE {phase}')
                weld_on = bool(cd.get('weld_on', 0))
                weave_pattern = cd.get('weave_pattern', '')
                if weave_pattern:
                    nominal_mm = np.array(cd.get('nominal_position', cd['position']), dtype=float) / 1000.0
                    off = np.linalg.norm(nominal_mm - self._live_tcp) * 1000.0
                    lines.append(
                        f'WEAVE {weave_pattern}  Lat {cd.get("weave_lateral", 0):+.2f} mm  '
                        f'Vert {cd.get("weave_vertical", 0):+.2f} mm  Off {off:.2f} mm'
                    )
                lines.append('WELD ON' if weld_on else 'WELD OFF')
                if weld_on:
                    lines.append(
                        f'ARC  Current {cd.get("current_pct", 0):.0f}%  Voltage {cd.get("voltage_pct", 0):.0f}%'
                    )

            if len(self.tcp_positions) > 1:
                lines.append(f'PATH {self._tcp_path_total_m:.3f} m')
                if len(self.tcp_positions) >= 2:
                    step_mm = np.linalg.norm(self.tcp_positions[-1] - self.tcp_positions[-2]) * 1000.0
                    interval_s = max(1, int(self.traj_interval_ms / self.traj_speed_mult)) / 1000.0
                    speed = step_mm / interval_s if interval_s > 0 else 0.0
                    lines.append(f'STEP {step_mm:.2f} mm  Speed {speed:.0f} mm/s')

            if len(self.traj_orientation_deltas) > 10:
                recent = self.traj_orientation_deltas[-10:]
                avg_c = float(np.mean(recent))
                max_c = float(np.max(recent))
                if max_c < 1:
                    qual = 'Excellent'
                elif max_c < 3:
                    qual = 'Very Good'
                elif max_c < 5:
                    qual = 'Good'
                elif max_c < 10:
                    qual = 'Fair'
                else:
                    qual = 'Poor'
                lines.append(f'ORIΔ avg {avg_c:.2f}°/step  max {max_c:.2f}°  {qual}')

        self._set_info_text('\n'.join(lines))

    def update_robot_display(self):
        if self.cleanup_in_progress:
            return
        try:
            with QMutexLocker(self.mutex):
                ja = self.current_joints.copy()

            positions, orientations = self.forward_kinematics(ja)
            tcp = positions[-1]
            self._live_tcp = tcp.copy()

            for index in range(len(positions) - 1):
                self._set_line_item(self._link_items[index], np.vstack([positions[index], positions[index + 1]]))
                self._set_scatter_item(self._joint_items[index], positions[index + 1], color=self._LINK_COLORS[index], size=10)

            if not self.tcp_positions:
                self.tcp_positions.append(tcp.copy())
            else:
                seg_len = float(np.linalg.norm(tcp - self.tcp_positions[-1]))
                if seg_len > 1e-6:
                    self.tcp_positions.append(tcp.copy())
                    self._tcp_segment_lengths.append(seg_len)
                    self._tcp_path_total_m += seg_len
            if len(self.tcp_positions) > 2000:
                excess = len(self.tcp_positions) - 2000
                if excess > 0:
                    if self._tcp_segment_lengths:
                        self._tcp_path_total_m -= sum(self._tcp_segment_lengths[:excess])
                        self._tcp_segment_lengths = self._tcp_segment_lengths[excess:]
                    self.tcp_positions = self.tcp_positions[-2000:]

            # Current cartesian dict (during playback)
            cd = None
            if self.traj_playing and self.traj_has_cartesian and self.traj_frame > 0:
                idx = self.traj_frame - 1
                if idx < len(self.traj_cartesian):
                    cd = self.traj_cartesian[idx]

            planned_pos = None
            if cd is not None:
                planned_pos = np.array(cd.get('nominal_position', cd['position']), dtype=float) / 1000.0
                self.traj_woven_trail.append(planned_pos.copy())
                self.traj_weld_flags.append(bool(cd.get('weld_on', 0)))
                if len(self.traj_woven_trail) > 2000:
                    self.traj_woven_trail = self.traj_woven_trail[-2000:]
                    self.traj_weld_flags = self.traj_weld_flags[-2000:]

            tcp_orientation = np.asarray(orientations[-1], dtype=float).reshape(-1) if len(orientations) > 0 else np.zeros(3, dtype=float)
            if tcp_orientation.shape != (3,):
                tcp_orientation = np.zeros(3, dtype=float)

            # Orientation continuity tracking
            if len(self.tcp_positions) > 1:
                if self._prev_orient is not None:
                    r1 = R_scipy.from_euler('xyz', self._prev_orient, degrees=True)
                    r2 = R_scipy.from_euler('xyz', tcp_orientation, degrees=True)
                    q1, q2 = r1.as_quat(), r2.as_quat()
                    if np.dot(q1, q2) < 0:
                        q2 = -q2
                    ang = 2 * np.arccos(np.abs(np.clip(np.dot(q1, q2), -1, 1)))
                    self.traj_orientation_deltas.append(np.degrees(ang))
                self._prev_orient = tcp_orientation.copy()
            else:
                self._prev_orient = tcp_orientation.copy()

            if cd is not None and len(self.traj_woven_trail) > 1:
                wp = np.array(self.traj_woven_trail)
                weld_cols = self._build_weld_path_colors(self.traj_weld_flags)
                self._set_line_item_with_colors(self._woven_path_item, wp, weld_cols)
                if len(self.tcp_positions) > 1:
                    tp = np.array(self.tcp_positions)
                    self._set_line_item(self._tcp_path_item, tp)
                if len(self.traj_woven_trail) > 30:
                    rw = np.array(self.traj_woven_trail[-30:])
                    recent_cols = self._build_weld_path_colors(self.traj_weld_flags[-30:])
                    self._set_line_item_with_colors(self._recent_woven_path_item, rw, recent_cols)
                else:
                    self._set_line_item(self._recent_woven_path_item, None)
                self._set_line_item(self._recent_path_item, None)
            elif len(self.tcp_positions) > 1:
                tp = np.array(self.tcp_positions)
                self._set_line_item(self._tcp_path_item, tp)
                if len(self.tcp_positions) > 30:
                    rp = np.array(self.tcp_positions[-30:])
                    self._set_line_item(self._recent_path_item, rp)
                else:
                    self._set_line_item(self._recent_path_item, None)
                self._set_line_item(self._woven_path_item, None)
                self._set_line_item(self._recent_woven_path_item, None)
            else:
                self._set_line_item(self._tcp_path_item, None)
                self._set_line_item(self._recent_path_item, None)
                self._set_line_item(self._woven_path_item, None)
                self._set_line_item(self._recent_woven_path_item, None)

            self._set_scatter_item(self._tcp_item, tcp, color=(1.0, 0.92, 0.23, 1.0), size=14)
            self._set_tcp_axes(tcp, tcp_orientation)
            self._update_info_panel(ja, tcp, orientations, cd)

            if self.traj_playing and self.traj_joints is not None:
                self._set_status_text(f"Playing: {self.traj_frame}/{len(self.traj_joints)}")
            elif not self._last_status_text:
                self._set_status_text("Ready")

        except Exception as e:
            if not self.cleanup_in_progress:
                print(f"Warning: 3D display error: {e}")

    # --------------------------------------------------------- Helpers
    def clear_tcp_path(self):
        if self.cleanup_in_progress:
            return
        with QMutexLocker(self.mutex):
            self.tcp_positions.clear()
            self._tcp_segment_lengths.clear()
            self._tcp_path_total_m = 0.0
            self.traj_woven_trail.clear()
            self.traj_weld_flags.clear()
            self.traj_orientation_deltas.clear()
        self.schedule_render()

    def reset_view(self):
        if not self.cleanup_in_progress and hasattr(self, 'view') and self.view is not None:
            self.view.setCameraPosition(distance=4.0, elevation=20, azimuth=45)

    def cleanup(self):
        """Safe cleanup for the GL visualiser."""
        self.cleanup_in_progress = True
        self.traj_timer.stop()
        self.render_timer.stop()
        try:
            self.tcp_positions = []
            self._tcp_segment_lengths = []
            self._tcp_path_total_m = 0.0
            self.traj_woven_trail = []
            self.current_joints = [0.0] * 6
            self.traj_orientation_deltas = []
            self._prev_orient = None
            if self._tcp_mount_item is not None:
                self._tcp_mount_item.setData(pos=np.zeros((2, 3)))
            for item in self._tcp_axis_items:
                item.setData(pos=np.zeros((2, 3)))
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
#  GoMotionThread – executes a saved path on the real robot via the
#  feedback daemon, reading joint-angle + path files line by line.
# ═══════════════════════════════════════════════════════════════════════════
class GoMotionThread(QThread):
    """Background thread that streams a pre-generated path to the robot."""
    target_update = QtSignal(dict)

    def __init__(self, gui_instance, loop_count=1):
        super().__init__()
        self.gui = gui_instance
        self.running = True
        self.paused = False
        self.loop_count = loop_count
        self.stream_idle_sleep_s = 0.005
        self.stream_enqueue_sleep_s = 0.002

    # ── helpers ──────────────────────────────────────────────────────────
    def _wait_for_condition(self, check_fn, description, tolerance, timeout):
        start = time.time()
        while time.time() - start < timeout:
            if check_fn():
                self.gui.log_event(f"Robot reached {description}", "SUCCESS")
                time.sleep(getattr(self.gui, 'stabilization_delay', 0.5))
                return True
            time.sleep(0.5)
        self.gui.log_event(f"Timeout waiting for {description}", "WARNING")
        return False

    def _wait_for_position_reached(self, target_joints, tolerance=0.07, timeout=10):
        def check():
            cur = getattr(self.gui, '_feedback_current_joints', None)
            if cur and len(cur) == len(target_joints):
                return all(abs(c - t) <= tolerance for c, t in zip(cur, target_joints))
            return False
        return self._wait_for_condition(check, "joint position", tolerance, timeout)

    # ── main loop ────────────────────────────────────────────────────────
    def run(self):
        if not self.gui.current_program_file or self.gui.current_program_file == "Untitled Program":
            self.gui.log_event("No program file to execute.", "WARNING")
            return

        # Derive joint-angle and path file names from the program file
        base = self.gui.current_program_file
        if base.endswith('.json'):
            base = base[:-5]
        # Prefer machine-bound KSM trajectories, keep legacy TXT fallback.
        jangles_file = base + "_joint_angles.ksm"
        if not os.path.exists(jangles_file):
            legacy = base + "_joint_angles.txt"
            if os.path.exists(legacy):
                jangles_file = legacy
            else:
                self.gui.log_event(f"File not found: {jangles_file}", "ERROR")
                return

        if jangles_file.lower().endswith('.ksm'):
            ok, msg = validate_ksm_metadata(read_ksm_metadata(jangles_file))
            if not ok:
                self.gui.log_event(
                    f"KSM verification failed: {msg}. "
                    "Please press 'Generate Path' to regenerate the trajectory.",
                    "ERROR",
                )
                return

        with open(jangles_file) as fj:
            raw_lines = [ln.strip() for ln in fj if ln.strip()]

        # Keep command lines and data lines; skip comments.
        lines = [ln for ln in raw_lines if not ln.startswith('#')]
        data_lines = [ln for ln in lines if not ln.startswith('---')]
        if not data_lines:
            self.gui.log_event("Trajectory has no data lines.", "ERROR")
            return

        # Determine the final joint angles for end-of-loop check
        final_joints = None
        for line in reversed(data_lines):
            line = line.strip()
            if line:
                try:
                    final_joints = [float(x) for x in line.split(',')[:6]]
                    break
                except (ValueError, IndexError):
                    continue

        for loop in range(self.loop_count):
            if not self.running:
                break
            self.gui.log_event(f"--- Loop {loop + 1}/{self.loop_count} ---", "PROGRAM")

            # Move to the starting position
            first = data_lines[0]
            try:
                parts = [x.strip() for x in first.split(',')]
                joints = [float(x) for x in parts[:6]]
                speed = 50
                grip = "Unlocked"
                tool = "Unlocked"
                self.gui.command_queue.put(['joint_angles'] + joints + [speed, grip, tool])
                self.gui.log_event("Moving to start position…", "PROGRAM")
                if not self._wait_for_position_reached(joints):
                    self.gui.log_event("Failed to reach start – aborting.", "ERROR")
                    return
            except (ValueError, IndexError):
                pass

            # Stream remaining waypoints
            for jl in data_lines[1:]:
                while self.paused:
                    time.sleep(0.1)
                if not self.running:
                    break

                # Respect look-ahead buffer
                buf = getattr(self.gui, 'look_ahead_buffer_size', 64)
                while self.gui.command_queue.qsize() >= buf:
                    if not self.running:
                        break
                    time.sleep(self.stream_idle_sleep_s)

                jl = jl.strip()
                try:
                    jp = [x.strip() for x in jl.split(',')]
                    joints = [float(x) for x in jp[:6]]
                    speed = 50
                    grip = "Unlocked"
                    tool = "Unlocked"

                    # KSM segment speed is embedded after weave columns.
                    # Use it when present to avoid flattening the planned profile.
                    if len(jp) >= 20:
                        try:
                            seg_speed = float(jp[19])
                            if seg_speed > 0.0:
                                speed = seg_speed
                        except (ValueError, IndexError):
                            pass

                    # KSM/TXT generated trajectory embeds Cartesian fields after phase:
                    # J1..J6, Phase, X, Y, Z, RX, RY, RZ, ...
                    if len(jp) >= 13:
                        self.target_update.emit({
                            'X': float(jp[7]),
                            'Y': float(jp[8]),
                            'Z': float(jp[9]),
                            'Roll': float(jp[10]),
                            'Pitch': float(jp[11]),
                            'Yaw': float(jp[12]),
                        })

                    self.gui.command_queue.put(['joint_angles'] + joints + [speed, grip, tool])
                except (ValueError, IndexError):
                    pass
                time.sleep(self.stream_enqueue_sleep_s)

            # Wait until the queue drains
            while not self.gui.command_queue.empty():
                if not self.running:
                    break
                time.sleep(self.stream_idle_sleep_s)

            # Wait for final position
            if self.running and final_joints:
                self.gui.log_event("Waiting for final position…", "PROGRAM")
                if not self._wait_for_position_reached(final_joints):
                    self.gui.log_event("Did not reach final position – aborting loop.", "WARNING")
                    break

        self.gui.log_event("Path execution finished.", "SUCCESS")

    def stop(self):
        self.running = False

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False


# ═════════════════════════════════════════════════════════════════════════════
#  MainWindow – Primary UI window
# ═════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinmatech Robotics Control")
        self.setGeometry(100, 100, 1200, 800)
        self.showMaximized()   # fill whatever display is connected
        
        # Robot state simulation
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.unwrapped_wrist = [0.0, 0.0, 0.0]  # Absolute J4, J5, J6 (accumulating rotations)
        self.last_fk_joints: Optional[List[float]] = None
        self.tcp_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._last_idle_tcp_pose = None
        self._last_control_pose = None
        self._pending_cartesian_fk_lock = None
        self._cart_axis_maps = {}
        self._cart_axis_limits = {
            'x': (-1200.0, 1200.0),
            'y': (-1200.0, 1200.0),
            'z': (-200.0, 1800.0),
            'rx': (-180.0, 180.0),
            'ry': (-180.0, 180.0),
            'rz': (-180.0, 180.0),
        }
        # Storage-based jog lookup (built while idle)
        self.axis_storage_dir = os.path.join(_project_root, "positions", "jog_maps")
        os.makedirs(self.axis_storage_dir, exist_ok=True)
        self.axis_storage_samples = 240          # 50–300 configurable
        self.axis_interp_samples = self.axis_storage_samples
        self.axis_storage_file = os.path.join(self.axis_storage_dir, "axis_maps_yaskawa.json")
        self.axis_storage_max_jump = 18.0        # max delta any joint between samples
        self.axis_storage_elbow_jump = 12.0      # stricter for J2/J3 continuity
        self._last_axis_jog_state = {'axis': None, 'joints': None}
        self.storage_idle_timer = QTimer()
        self.storage_idle_timer.setSingleShot(True)
        self.idle_pos_deadband_mm = 0.25
        self.idle_ori_deadband_deg = 0.35
        self.log_hardware_command_success = False
        self.use_quaternion_wrist_scoring = True
        
        # Jog data persistence
        self.jog_data_file = "jog_data.json"
        
        # Program data management
        self.current_program_file = "Untitled Program"
        self.program_modified = False
        self.positions_base_dir = "positions"  # Base directory for all programs
        
        # Welding state
        self.welding_enabled = False  # Weld parameter setting for program entries
        self.torch_active = False     # Actual torch state (only active in JOG mode)
        self.movement_active = False  # Track if robot is currently moving
        self._traj_weld_sync_active = False
        self._preplay_welding_enabled = None
        self._preplay_torch_active = None
        self.movement_timeout_timer = QTimer()  # Timer to detect movement stop
        self.movement_timeout_timer.setSingleShot(True)
        self.movement_timeout_timer.timeout.connect(self.on_movement_stopped)
        self.cartesian_update_interval_ms = 15
        
        # Jog button long press timers
        self.jog_timers = {}
        self.jog_active = {}
        self.jog_button_meta = {}
        self.jog_press_time = {}
        self.jog_hold_threshold_s = 0.18
        self.jog_initial_delay_ms = 180
        self.jog_repeat_interval_ms = 40
        
        # Jog step sizes
        self.joint_step_size = 1.0  # degrees
        self.cartesian_step_size = 1.0  # mm for position, degrees for orientation
        self.movement_speed = 50.0  # mm/s or deg/s for program table
        self.motion_frame_interval_ms = 40  # traversal frame interval (~25 Hz)
        
        # Motion type selection
        self.current_motion_type = "LINEAR"  # LINEAR / CURVE / P2P
        
        # Path generation
        self.path_generator = PathGenerator()
        
        # Record positions (1-16) for quick access
        self.record_positions = {}  # Dict to store positions for records 1-16
        self.home_position = None   # Home position storage
        
        # GUI update throttling for cartesian mode
        self.cartesian_update_timer = QTimer()
        self.cartesian_update_timer.setSingleShot(True)
        self.cartesian_update_timer.timeout.connect(self.update_cartesian_display)
        
        # Toolbar button states
        self.button_states = {
            'connect': False,
            'jog': False,  # Disabled until connected
            'teach': False,
            'run': False,
            'abort': False
        }
        
        # Initialize 3D robot visualizer before UI setup
        try:
            self.robot_visualizer = Robot3DVisualizer()
            # Connect trajectory playback signals
            self.robot_visualizer.frame_changed.connect(self._on_traj_frame_changed)
            self.robot_visualizer.playback_finished.connect(self._on_traj_finished)
        except Exception as e:
            print(f"Warning: Could not initialize 3D visualizer: {e}")
            self.robot_visualizer = None

        # ── Transition motion mode (P2P vs LINEAR) ──
        self.transition_mode = "P2P"  # "P2P" or "LINEAR"

        # ── P2P interpolated motion state ──
        self.p2p_active = False
        self.p2p_start_joints = None
        self.p2p_target_joints = None
        self.p2p_target_tcp = None
        self.p2p_total_steps = 0
        self.p2p_current_step = 0
        self.p2p_timer = QTimer()
        self.p2p_timer.timeout.connect(self._p2p_tick)

        # ── Linear (Cartesian) interpolated motion state ──
        self.linear_active = False
        self.linear_start_tcp = None
        self.linear_target_tcp = None
        self.linear_target_joints = None
        self.linear_on_complete = None
        self.linear_total_steps = 0
        self.linear_current_step = 0
        self.linear_slerp = None
        self.linear_timer = QTimer()
        self.linear_timer.timeout.connect(self._linear_tick)

        # ── Physical jog preplanner ──
        self.physical_jog_preplan_enabled = True
        self.physical_jog_plan_rate_hz = 40.0
        self.physical_jog_plan_debounce_ms = 120
        self.jog_knowledge_samples_per_direction = 120
        self.jog_knowledge_position_span_mm = 120.0
        self.jog_knowledge_orientation_span_deg = 60.0
        self.jog_preplan_thread = None
        self._pending_jog_preplan_request = None
        self._jog_preplan_request_seq = 0
        self._latest_jog_preplan = None
        self._latest_jog_preplan_file = None
        self.jog_preplan_timer = QTimer()
        self.jog_preplan_timer.setSingleShot(True)
        self.jog_preplan_timer.timeout.connect(self._dispatch_jog_preplan)
        
        # ── Persistent logging ──
        self.log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self._log_buffer: list = []     # plain-text lines for file output
        self._log_throttle_state = {}
        self._log_prefix_throttle_seconds = {
            "Control loop overrun:": 5.0,
            "FK computation error:": 5.0,
            "Hardware disconnected:": 10.0,
            "Hardware connection unavailable": 10.0,
            "Pendant disconnected": 10.0,
        }
        # Console append batching: batch multiple logs before triggering repaint
        self._console_batch_buffer: list = []
        self._console_batch_timer = QTimer()
        self._console_batch_timer.setSingleShot(True)
        self._console_batch_timer.timeout.connect(self._flush_console_batch)
        self._console_batch_timeout_ms = 50  # Batch for up to 50ms before flushing
        self._purge_old_logs(max_age_days=30)

        # ── Command queue and control threads ──
        self.pendant_connected = False
        self._pendant_prev = ""          # previous command string for edge detection
        self._pendant_cart_lock = None    # sticky cartesian axis during jog
        self.stabilization_delay = 0.5      # seconds – wait after reaching target
        self.look_ahead_buffer_size = 64    # keep queue primed to avoid transition stutter
        self.step_size = 0.06               # mm – smallest cartesian increment
        self._feedback_current_joints = None  # populated by feedback polling
        self._cart_jog_elbow_branch = None    # lock elbow branch during cartesian jog bursts

        # Initialize command queue
        self.command_queue = ThreadSafeCommandQueue()
        
        # Initialize HardwareThread FIRST (so ControlThread can connect to it)
        self.hardware_thread = HardwareThread()
        self.hardware_thread.connected.connect(self._on_hardware_connected)
        self.hardware_thread.feedback_received.connect(self._on_hardware_feedback)
        self.hardware_thread.command_sent.connect(self._on_hardware_command_sent)
        self.hardware_thread.log_message.connect(self._on_hardware_log)
        self.hardware_thread.error_occurred.connect(self._on_hardware_error)
        self.hardware_thread.start()
        
        # Initialize ControlThread (100 Hz fixed-rate loop)
        self.control_thread = ControlThread(self.command_queue)
        self.control_thread.log_message.connect(self._on_control_log)
        self.control_thread.movement_started.connect(self._on_movement_started)
        self.control_thread.movement_completed.connect(self._on_movement_completed)
        self.control_thread.joint_targets.connect(self.hardware_thread.receive_joint_targets)
        self.control_thread.cartesian_state.connect(self._on_cartesian_state_update)
        self.control_thread.halt_requested.connect(self._on_halt_requested)
        self.control_thread.pause_requested.connect(self._on_pause_requested)
        self.control_thread.resume_requested.connect(self._on_resume_requested)
        self.hardware_thread.feedback_received.connect(self._on_hardware_feedback_for_control)
        self.control_thread.start()

        # Initialize pendant thread (already QThread-based)
        self.pendant_thread = PendantConnectionThread()
        self.pendant_thread.connection_status.connect(self._on_pendant_status)
        self.pendant_thread.command_received.connect(self._on_pendant_command)
        self.pendant_thread.log_message.connect(self._on_pendant_log)
        self.pendant_thread.start()

        # Set up the UI
        self.setup_ui()

        # Load saved jog data
        self.load_jog_data()
        
        # Initialize kinematic worker thread
        self.setup_kinematic_worker()
        
        # Setup robot state polling
        self.setup_robot_polling()
        
        # Initialize UI component states
        self.update_ui_components()
        
        # Request initial FK calculation to populate TCP display
        # Use a timer to ensure the kinematic worker thread is fully started
        QTimer.singleShot(100, self.request_initial_fk)
        
        # Initialize console with startup message
        QTimer.singleShot(200, lambda: self.log_event("Kinmatech Robotics application started", "SUCCESS"))
        
        # Initialize UI status displays
        QTimer.singleShot(300, self.initialize_ui_status)
        
    def setup_ui(self):
        """Set up the main user interface"""
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout (vertical)
        main_layout = QVBoxLayout(central_widget)
        
        # Create top section with 3 columns
        top_layout = QHBoxLayout()
        
        # Left column - JogPanel
        self.jog_panel = QGroupBox("Jog Panel")
        self.jog_panel.setMinimumWidth(380)
        self.setup_jog_panel()
        
        # Center column - StatusPanel with 3D Visualization
        self.status_panel = QGroupBox("Status Panel")
        self.status_panel.setMinimumWidth(400)  # Increased width for 3D view
        self.setup_status_panel()
        
        # Right column - WeldPanel
        self.weld_panel = QGroupBox("Weld Panel")
        self.weld_panel.setMinimumWidth(300)
        self.setup_weld_panel()
        
        # Add panels to top layout
        top_layout.addWidget(self.jog_panel)
        top_layout.addWidget(self.status_panel)
        
        # Right column - combine weld panel and console
        right_column = QWidget()
        right_layout = QVBoxLayout(right_column)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)
        
        # Add weld panel to right column
        right_layout.addWidget(self.weld_panel, 0)  # No stretch, compact
        
        # Add console panel
        self.console_panel = QGroupBox("Event Console")
        self.console_panel.setMinimumHeight(150)
        self.setup_console_panel()
        right_layout.addWidget(self.console_panel, 1)  # Stretch to fill remaining space
        
        top_layout.addWidget(right_column)
        
        # Bottom section - Program Table with controls
        program_section = QWidget()
        program_layout = QVBoxLayout(program_section)
        
        # Program table header with filename and controls
        program_header = QHBoxLayout()
        program_header.setSpacing(2)  # Reduce button spacing
        program_header.setContentsMargins(2, 2, 2, 2)  # Reduce margins
        self.program_title_label = QLabel(f"Program: {self.current_program_file}")
        self.program_title_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        program_header.addWidget(self.program_title_label)
        
        # Add spacing between title and buttons
        program_header.addSpacing(10)
        
        # Table control buttons
        
        # Path and execution buttons (before Add Position)
        self.generate_path_btn = QPushButton("Generate Path")
        self.generate_path_btn.setStatusTip("Generate optimized path from program points")
        self.generate_path_btn.setMaximumWidth(135)
        self.generate_path_btn.clicked.connect(self.generate_path)
        program_header.addWidget(self.generate_path_btn)
        
        self.play_btn = QPushButton("Play")
        self.play_btn.setStatusTip("Execute the robot program")
        self.play_btn.setMaximumWidth(60)
        self.play_btn.clicked.connect(self.play_program)
        program_header.addWidget(self.play_btn)
        
        self.dry_run_btn = QPushButton("Dry Run")
        self.dry_run_btn.setStatusTip("Simulate program execution without welding")
        self.dry_run_btn.setMaximumWidth(75)
        self.dry_run_btn.clicked.connect(self.dry_run_program)
        program_header.addWidget(self.dry_run_btn)
        
        # Position manipulation buttons
        self.add_row_btn = QPushButton("Add Current Position")
        self.add_row_btn.setStatusTip("Add current robot position and weld settings to program")
        self.add_row_btn.setMaximumWidth(160)
        self.add_row_btn.clicked.connect(self.add_current_position_to_program)
        program_header.addWidget(self.add_row_btn)
        
        # Timer and trigger buttons (after Add Position)
        self.add_timer_btn = QPushButton("Add Timer")
        self.add_timer_btn.setStatusTip("Add timer command to program")
        self.add_timer_btn.setMaximumWidth(100)
        self.add_timer_btn.clicked.connect(self.add_timer_to_program)
        program_header.addWidget(self.add_timer_btn)
        
        self.add_trigger_btn = QPushButton("Add Trigger")
        self.add_trigger_btn.setStatusTip("Add trigger command to program")
        self.add_trigger_btn.setMaximumWidth(110)
        self.add_trigger_btn.clicked.connect(self.add_trigger_to_program)
        program_header.addWidget(self.add_trigger_btn)
        
        self.add_home_btn = QPushButton("Add Home")
        self.add_home_btn.setStatusTip("Add go-to-home command to program")
        self.add_home_btn.setMaximumWidth(105)
        self.add_home_btn.clicked.connect(self.add_home_to_program)
        program_header.addWidget(self.add_home_btn)
        
        self.delete_row_btn = QPushButton("Delete Row")
        self.delete_row_btn.setStatusTip("Delete selected row from program")
        self.delete_row_btn.setMaximumWidth(100)
        self.delete_row_btn.clicked.connect(self.delete_selected_row)
        program_header.addWidget(self.delete_row_btn)
        
        self.save_btn = QPushButton("Save")
        self.save_btn.setStatusTip("Save program to current file")
        self.save_btn.setMaximumWidth(60)
        self.save_btn.clicked.connect(self.save_program)
        program_header.addWidget(self.save_btn)
        
        self.save_as_btn = QPushButton("Save As")
        self.save_as_btn.setStatusTip("Save program with a new filename")
        self.save_as_btn.setMaximumWidth(75)
        self.save_as_btn.clicked.connect(self.save_program_as)
        program_header.addWidget(self.save_as_btn)
        
        self.load_btn = QPushButton("Load")
        self.load_btn.setStatusTip("Load program from file")
        self.load_btn.setMaximumWidth(60)
        self.load_btn.clicked.connect(self.load_program)
        program_header.addWidget(self.load_btn)
        
        self.new_btn = QPushButton("New")
        self.new_btn.setStatusTip("Create new program")
        self.new_btn.setMaximumWidth(60)
        self.new_btn.clicked.connect(self.new_program)
        program_header.addWidget(self.new_btn)
        
        # Add stretch at the end to push buttons to the left
        program_header.addStretch()
        
        program_layout.addLayout(program_header)
        
        # Program table
        self.program_table = QTableView()
        self.program_table.setMinimumHeight(200)
        program_layout.addWidget(self.program_table)
        
        # Create and set up the program table model
        self.setup_program_table()
        
        # Add sections to main layout
        main_layout.addLayout(top_layout, 2)  # Takes 2/3 of space
        main_layout.addWidget(program_section, 1)  # Takes 1/3 of space
        
        # Set up toolbar
        self.setup_toolbar()
        
        # Set up status bar
        self.setup_status_bar()
        
    def setup_toolbar(self):
        """Set up the top toolbar with toggle control buttons"""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        # Create button style for active/inactive states
        # Compact sizing for 7-10" pendant displays
        button_style = """
        QPushButton {
            padding: 3px 6px;
            font-size: 11px;
            font-weight: bold;
            border: 2px solid #ccc;
            border-radius: 4px;
            background-color: #f0f0f0;
            color: black;
            min-width: 46px;
        }
        QPushButton:hover {
            background-color: #e0e0e0;
            color: black;
        }
        QPushButton:checked {
            background-color: #4CAF50;
            color: white;
            border-color: #45a049;
        }
        QPushButton:pressed {
            background-color: #45a049;
        }
        """
        
        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.setCheckable(True)
        self.connect_button.setChecked(self.button_states['connect'])
        self.connect_button.setStyleSheet(button_style)
        self.connect_button.setStatusTip("Connect to robot")
        self.connect_button.clicked.connect(lambda: self.toggle_mode('connect'))
        toolbar.addWidget(self.connect_button)
        
        toolbar.addSeparator()
        
        # Jog button (disabled until connected)
        self.jog_button = QPushButton("Jog")
        self.jog_button.setCheckable(True)
        self.jog_button.setChecked(self.button_states['jog'])
        self.jog_button.setEnabled(False)  # Disabled until connected
        self.jog_button.setStyleSheet(button_style)
        self.jog_button.setStatusTip("Enable jogging mode")
        self.jog_button.clicked.connect(lambda: self.toggle_mode('jog'))
        toolbar.addWidget(self.jog_button)
        
        # Teach button
        self.teach_button = QPushButton("Teach")
        self.teach_button.setCheckable(True)
        self.teach_button.setChecked(self.button_states['teach'])
        self.teach_button.setEnabled(False)  # Disabled until connected
        self.teach_button.setStyleSheet(button_style)
        self.teach_button.setStatusTip("Enable teaching mode")
        self.teach_button.clicked.connect(lambda: self.toggle_mode('teach'))
        toolbar.addWidget(self.teach_button)
        
        toolbar.addSeparator()
        
        # Run button
        self.run_button = QPushButton("Run")
        self.run_button.setCheckable(True)
        self.run_button.setChecked(self.button_states['run'])
        self.run_button.setEnabled(False)  # Disabled until connected
        self.run_button.setStyleSheet(button_style)
        self.run_button.setStatusTip("Run program")
        self.run_button.clicked.connect(lambda: self.toggle_mode('run'))
        toolbar.addWidget(self.run_button)
        
        # Abort button with special red styling (compact)
        abort_button_style = """
        QPushButton {
            padding: 3px 6px;
            font-size: 11px;
            font-weight: bold;
            border: 2px solid #cc0000;
            border-radius: 4px;
            background-color: #ff4444;
            color: white;
            min-width: 46px;
        }
        QPushButton:hover {
            background-color: #ff6666;
        }
        QPushButton:pressed {
            background-color: #0066cc;
            border-color: #004499;
        }
        QPushButton:disabled {
            background-color: #cccccc;
            color: #666666;
            border-color: #aaaaaa;
        }
        """
        
        self.abort_button = QPushButton("Abort")
        self.abort_button.setStyleSheet(abort_button_style)
        self.abort_button.setStatusTip("Emergency stop")
        self.abort_button.setEnabled(False)  # Disabled until connected
        self.abort_button.clicked.connect(self.handle_abort)
        toolbar.addWidget(self.abort_button)
        
        toolbar.addSeparator()

        # Transition mode selector (P2P vs Linear)
        transition_label = QLabel(" Tr: ")
        transition_label.setStyleSheet("color: #ccc; font-weight: bold; font-size: 11px;")
        toolbar.addWidget(transition_label)
        self.transition_combo = QComboBox()
        self.transition_combo.addItems(["P2P", "Linear"])
        self.transition_combo.setCurrentText(self.transition_mode)
        self.transition_combo.setToolTip(
            "P2P: joint-space interpolation (fast, unpredictable TCP path)\n"
            "Linear: straight-line TCP motion (safer, may fail near singularities)"
        )
        self.transition_combo.currentTextChanged.connect(self._on_transition_mode_changed)
        self.transition_combo.setStyleSheet("""
            QComboBox {
                background-color: #2a2a2a;
                color: #00ff88;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 2px 4px;
                min-width: 44px;
                font-size: 11px;
                font-weight: bold;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #2a2a2a;
                color: #00ff88;
                selection-background-color: #444;
            }
        """)
        toolbar.addWidget(self.transition_combo)

        # Speed override selector
        ovr_label = QLabel(" Ovr: ")
        ovr_label.setStyleSheet("color: #ccc; font-weight: bold; font-size: 11px;")
        toolbar.addWidget(ovr_label)
        self.speed_override_combo = QComboBox()
        self.speed_override_combo.addItems(["25%", "50%", "75%", "100%"])
        self.speed_override_combo.setCurrentText("100%")
        self.speed_override_combo.setToolTip(
            "Global speed override \u2014 scales ALL motions\n"
            "(P2P, Linear, trajectory playback)")
        self.speed_override_combo.currentTextChanged.connect(
            self._on_speed_override_changed)
        self.speed_override_combo.setStyleSheet("""
            QComboBox {
                background-color: #2a2a2a;
                color: #ffcc00;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 2px 4px;
                min-width: 38px;
                font-size: 11px;
                font-weight: bold;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #2a2a2a;
                color: #ffcc00;
                selection-background-color: #444;
            }
        """)
        toolbar.addWidget(self.speed_override_combo)
        self.speed_override_factor = 1.0

        # Move to Position button
        self.move_to_position_button = QPushButton("MoveTo")
        self.move_to_position_button.setStyleSheet(button_style)
        self.move_to_position_button.setStatusTip("Move robot to selected position from program table")
        self.move_to_position_button.setEnabled(False)  # Disabled until connected
        self.move_to_position_button.clicked.connect(self.move_to_selected_position)
        toolbar.addWidget(self.move_to_position_button)
        
        # Record dropdown (1-16 with colors)
        self.record_dropdown = QComboBox()
        self.setup_record_dropdown()
        self.record_dropdown.setStatusTip("Select record position (1-16)")
        self.record_dropdown.setEnabled(False)  # Disabled until connected
        toolbar.addWidget(self.record_dropdown)
        
        # Set Record button
        self.set_record_button = QPushButton("SetRec")
        self.set_record_button.setStyleSheet(button_style)
        self.set_record_button.setStatusTip("Set current position to selected record (NO collision avoidance - USE WITH CAUTION!)")
        self.set_record_button.setEnabled(False)  # Disabled until connected
        self.set_record_button.clicked.connect(self.set_record_position)
        toolbar.addWidget(self.set_record_button)
        
        # Go Record button
        self.go_record_button = QPushButton("GoRec")
        self.go_record_button.setStyleSheet(button_style)
        self.go_record_button.setStatusTip("Move robot to selected record position (NO collision avoidance - USE WITH CAUTION!)")
        self.go_record_button.setEnabled(False)  # Disabled until connected
        self.go_record_button.clicked.connect(self.go_to_selected_record)
        toolbar.addWidget(self.go_record_button)
        
        toolbar.addSeparator()
        
        # Set New Home button
        self.set_home_button = QPushButton("SetHome")
        self.set_home_button.setStyleSheet(button_style)
        self.set_home_button.setStatusTip("Set current position as new home position")
        self.set_home_button.setEnabled(False)  # Disabled until connected
        self.set_home_button.clicked.connect(self.set_new_home_position)
        toolbar.addWidget(self.set_home_button)
        
        # Go Home button
        self.go_home_button = QPushButton("GoHome")
        self.go_home_button.setStyleSheet(button_style)
        self.go_home_button.setStatusTip("Move robot to home position")
        self.go_home_button.setEnabled(False)  # Disabled until connected
        self.go_home_button.clicked.connect(self.go_to_home_position)
        toolbar.addWidget(self.go_home_button)
        
    def setup_status_bar(self):
        """Set up the bottom status bar"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready - Connect to enable controls")
        
    def toggle_mode(self, mode_name):
        """Toggle toolbar button mode and update states with Connect logic"""
        if mode_name == 'connect':
            # Connect is independent toggle
            self.button_states['connect'] = not self.button_states['connect']
            self.connect_button.setChecked(self.button_states['connect'])
            
            # Enable/disable other buttons based on connection state
            connected = self.button_states['connect']
            self.jog_button.setEnabled(connected)
            self.teach_button.setEnabled(connected)
            self.run_button.setEnabled(connected)
            self.abort_button.setEnabled(connected)
            
            # Log connection state change
            if connected:
                self.log_event("Robot connection established", "SUCCESS")
            else:
                self.log_event("Robot disconnected", "WARNING")
            
            # If disconnecting, reset all operational modes
            if not connected:
                self.button_states['jog'] = False
                self.button_states['teach'] = False
                self.button_states['run'] = False
                self.jog_button.setChecked(False)
                self.teach_button.setChecked(False)
                self.run_button.setChecked(False)
                
        elif mode_name in ['jog', 'teach', 'run']:
            # Only allow if connected
            if not self.button_states['connect']:
                return
                
            # Jog and Teach are mutually exclusive, but can be toggled off
            if mode_name in ['jog', 'teach']:
                # Toggle the selected mode
                new_state = not self.button_states[mode_name]
                
                if new_state:
                    # If enabling jog or teach, disable the other
                    if mode_name == 'jog':
                        self.button_states['teach'] = False
                        self.teach_button.setChecked(False)
                        self.log_event("Jog mode activated", "JOG")
                        # Check if weld button is on and inform user about torch behavior
                        if self.welding_enabled:
                            self.log_event("Weld enabled - torch will activate during movement", "WELD")
                            self.update_torch_for_movement()
                    else:  # teach
                        self.button_states['jog'] = False
                        self.jog_button.setChecked(False)
                        # Safety: Deactivate torch and stop movement detection when switching to teach mode
                        if self.torch_active:
                            self.torch_active = False
                            self.log_event("TORCH DEACTIVATED - Switched to TEACH mode (weld button now for program parameters only)", "WELD")
                        self.movement_active = False
                        self.movement_timeout_timer.stop()
                        self.log_event("Teach mode activated - program editing enabled", "PROGRAM")
                else:
                    if mode_name == 'jog':
                        # Deactivate torch and stop movement detection when leaving jog mode
                        if self.torch_active:
                            self.torch_active = False
                            self.log_event("TORCH DEACTIVATED - Jog mode disabled", "WELD")
                        self.movement_active = False
                        self.movement_timeout_timer.stop()
                        self.log_event("Jog mode deactivated", "JOG")
                    else:  # teach
                        self.log_event("Teach mode deactivated - program editing disabled", "PROGRAM")
                
                self.button_states[mode_name] = new_state
                
                # Update button states
                self.jog_button.setChecked(self.button_states['jog'])
                self.teach_button.setChecked(self.button_states['teach'])
            
            elif mode_name == 'run':
                # Run is independent toggle
                new_state = not self.button_states['run']
                self.button_states['run'] = new_state
                self.run_button.setChecked(new_state)
                
                if new_state:
                    self.log_event("Run mode activated - ready for program execution", "SUCCESS")
                else:
                    self.log_event("Run mode deactivated", "INFO")
        
        # Update UI components based on new states
        self.update_ui_components()
        
        # Update status based on active modes
        active_modes = [mode for mode, active in self.button_states.items() 
                       if active and mode != 'abort']
        if active_modes:
            status_msg = f"Mode: {', '.join(active_modes).title()}"
            self.status_bar.showMessage(status_msg)
        else:
            self.status_bar.showMessage("Ready")
            
        # Update torch status display
        self.update_torch_status_display()
        
    def update_torch_status_display(self):
        """Update the torch status display in the status panel"""
        if self.torch_active:
            self.torch_label.setText("Torch: ACTIVE")
            self.torch_label.setStyleSheet("color: #FF4444; font-weight: bold;")  # Red
        elif self.welding_enabled:
            jog_mode = self.button_states.get('jog', False)
            teach_mode = self.button_states.get('teach', False)
            if jog_mode and not self.movement_active:
                self.torch_label.setText("Torch: READY")
                self.torch_label.setStyleSheet("color: #FFAA00; font-weight: bold;")  # Orange
            elif teach_mode:
                self.torch_label.setText("Torch: PARAM ON")
                self.torch_label.setStyleSheet("color: #FFAA00;")  # Orange
            else:
                self.torch_label.setText("Torch: PARAM ON")
                self.torch_label.setStyleSheet("color: #666666;")  # Gray
        else:
            self.torch_label.setText("Torch: OFF")
            self.torch_label.setStyleSheet("color: #666666;")  # Gray
    
    def on_movement_started(self):
        """Called when robot movement starts"""
        if not self.movement_active:
            self.movement_active = True
            # Check if torch should be activated
            self.update_torch_for_movement()
        
        # Reset the timeout timer (but don't start it here to avoid conflicts with jog timing)
        self.movement_timeout_timer.stop()
        
    def on_movement_stopped(self):
        """Called when robot movement stops (timeout reached)"""
        if self.movement_active:
            self.movement_active = False
            # Always deactivate torch when movement stops
            if self.torch_active:
                self.torch_active = False
                self.log_event("TORCH OFF - Movement stopped", "WELD")
                self.update_torch_status_display()
                
    def update_torch_for_movement(self):
        """Update torch state based on movement and mode conditions"""
        jog_mode_active = self.button_states.get('jog', False)
        
        if jog_mode_active and self.welding_enabled and self.movement_active:
            if not self.torch_active:
                self.torch_active = True
                self.log_event("TORCH ON - Movement started in JOG mode", "WELD")
                self.update_torch_status_display()
        elif self.torch_active:
            self.torch_active = False
            reason = "Movement stopped" if not self.movement_active else "Mode/weld conditions changed"
            self.log_event(f"TORCH OFF - {reason}", "WELD")
            self.update_torch_status_display()
            
    def handle_abort(self):
        """Handle emergency abort \u2014 stops ALL motion immediately."""
        self.log_event("EMERGENCY ABORT ACTIVATED - All operations stopped", "ERROR")
        
        # Emergency torch shutdown
        if self.torch_active:
            self.torch_active = False
            self.log_event("EMERGENCY: TORCH DEACTIVATED", "ERROR")

        # \u2500\u2500 Stop P2P motion \u2500\u2500
        if self.p2p_active:
            self.p2p_timer.stop()
            self.p2p_active = False
            self.log_event("P2P motion aborted", "ERROR")

        # \u2500\u2500 Stop Linear motion \u2500\u2500
        if self.linear_active:
            self.linear_timer.stop()
            self.linear_active = False
            self.log_event("Linear motion aborted", "ERROR")

        # \u2500\u2500 Stop trajectory playback \u2500\u2500
        viz = getattr(self, 'robot_visualizer', None)
        if viz is not None and getattr(viz, 'traj_playing', False):
            viz.stop_playback()
            self.log_event("Trajectory playback aborted", "ERROR")

        # Emergency stop should not restore pre-playback weld state.
        self._traj_weld_sync_active = False
        self._preplay_welding_enabled = None
        self._preplay_torch_active = None

        # \u2500\u2500 Stop all jog timers \u2500\u2500
        for key, timer in self.jog_timers.items():
            if timer is not None:
                timer.stop()
        self.jog_active.clear()
            
        # Stop movement tracking
        self.movement_active = False
        self.movement_timeout_timer.stop()
        
        # Reset all operational modes
        self.button_states['jog'] = False
        self.button_states['teach'] = False
        self.button_states['run'] = False
        
        # Update button states
        self.jog_button.setChecked(False)
        self.teach_button.setChecked(False)
        self.run_button.setChecked(False)
        
        # Update UI components
        self.update_ui_components()
        
        # Update status
        self.status_bar.showMessage("EMERGENCY STOP - All operations aborted")
        
        # Clear the command queue
        if hasattr(self, 'command_queue'):
            self.command_queue.clear()
        
        # Here you would add actual robot stop commands

    # ── Control thread signal handlers ───────────────────────
    @pyqtSlot(str, str)
    def _on_control_log(self, message: str, level: str):
        """Handle log messages from ControlThread."""
        self.log_event(message, level)
    
    @pyqtSlot(str, dict)
    def _on_movement_started(self, movement_type: str, details: dict):
        """Handle movement start signal from ControlThread."""
        self.log_event(f"Movement started: {movement_type}", "INFO")
        # TODO: Update UI to show movement in progress
    
    @pyqtSlot(str)
    def _on_movement_completed(self, command_type: str):
        """Handle movement completion signal from ControlThread."""
        self.log_event(f"Movement completed: {command_type}", "SUCCESS")
        # TODO: Update UI to show movement complete
    
    @pyqtSlot()
    def _on_halt_requested(self):
        """Handle HALT request from ControlThread."""
        self.log_event("HALT signal received", "ERROR")
        # Forward to hardware thread
        if hasattr(self, 'hardware_thread'):
            self.hardware_thread.send_halt()
    
    @pyqtSlot()
    def _on_pause_requested(self):
        """Handle PAUSE request from ControlThread."""
        self.log_event("PAUSE signal received", "WARNING")
        # Forward to hardware thread
        if hasattr(self, 'hardware_thread'):
            self.hardware_thread.send_pause()
    
    @pyqtSlot()
    def _on_resume_requested(self):
        """Handle RESUME request from ControlThread."""
        self.log_event("RESUME signal received", "SUCCESS")
        # Forward to hardware thread
        if hasattr(self, 'hardware_thread'):
            self.hardware_thread.send_resume()
    
    # ── Hardware thread signal handlers ──────────────────────
    @pyqtSlot(bool, str)
    def _on_hardware_connected(self, is_connected: bool, status_msg: str):
        """Handle hardware connection status changes."""
        if is_connected:
            self.log_event(f"Hardware connected: {status_msg}", "SUCCESS")
            if hasattr(self, 'hardware_status_label'):
                self.hardware_status_label.setText("Hardware: Connected")
                self.hardware_status_label.setStyleSheet("color: #00cc00; font-weight: bold;")
        else:
            self.log_event(f"Hardware disconnected: {status_msg}", "WARNING")
            if hasattr(self, 'hardware_status_label'):
                self.hardware_status_label.setText("Hardware: Disconnected")
                self.hardware_status_label.setStyleSheet("color: #888888; font-weight: bold;")
    
    @pyqtSlot(dict)
    def _on_hardware_feedback(self, feedback: dict):
        """Handle feedback data from hardware thread."""
        # Update internal state (thread-safe via signals)
        if 'joints' in feedback:
            self._feedback_current_joints = feedback['joints']
        # Optionally update UI display
    
    @pyqtSlot(dict)
    def _on_hardware_feedback_for_control(self, feedback: dict):
        """
        Forward hardware feedback to ControlThread for state synchronization.
        The ControlThread uses this to keep track of current position.
        
        Args:
            feedback: Dict with 'joints', 'pose', 'gripper', 'tool', 'timestamp'
        """
        if feedback and 'joints' in feedback:
            self.control_thread.update_hardware_feedback(feedback['joints'])
    
    @pyqtSlot(dict)
    def _on_cartesian_state_update(self, state: dict):
        """
        Handle cartesian state updates from ControlThread (100 Hz).
        Updates UI display of current TCP position/orientation.
        
        Args:
            state: Dict with keys 'x', 'y', 'z', 'roll', 'pitch', 'yaw'
        """
        try:
            if hasattr(self, 'tcp_pos_x_display'):
                x = float(state.get('x', 0.0))
                y = float(state.get('y', 0.0))
                z = float(state.get('z', 0.0))
                roll = float(state.get('roll', 0.0))
                pitch = float(state.get('pitch', 0.0))
                yaw = float(state.get('yaw', 0.0))

                if self._last_control_pose is not None:
                    px, py, pz, pr, pp, pyaw = self._last_control_pose
                    roll = self._unwrap_angle_near(roll, pr)
                    pitch = self._unwrap_angle_near(pitch, pp)
                    yaw = self._unwrap_angle_near(yaw, pyaw)
                    pos_delta = max(abs(x - px), abs(y - py), abs(z - pz))
                    ori_delta = max(abs(roll - pr), abs(pitch - pp), abs(yaw - pyaw))
                    if pos_delta < self.idle_pos_deadband_mm and ori_delta < self.idle_ori_deadband_deg:
                        x, y, z, roll, pitch, yaw = self._last_control_pose

                self._last_control_pose = (x, y, z, roll, pitch, yaw)

                # Update TCP position display (only update periodically to avoid spam)
                if not hasattr(self, '_cartesian_update_count'):
                    self._cartesian_update_count = 0
                self._cartesian_update_count += 1
                
                if self._cartesian_update_count % 10 == 0:  # Update every 100ms
                    self.tcp_pos_x_display.setValue(x)
                    self.tcp_pos_y_display.setValue(y)
                    self.tcp_pos_z_display.setValue(z)
                    self.tcp_roll_display.setValue(roll)
                    self.tcp_pitch_display.setValue(pitch)
                    self.tcp_yaw_display.setValue(yaw)
                    self._cartesian_update_count = 0
        except Exception as e:
            self.log_event(f"Error updating cartesian state: {str(e)}", "WARNING")
    
    @pyqtSlot(str, bool)
    def _on_hardware_command_sent(self, command_type: str, success: bool):
        """Handle command execution result from hardware thread."""
        if success and not self.log_hardware_command_success:
            return
        status = "success" if success else "failed"
        self.log_event(f"Hardware command {command_type}: {status}", 
                      "SUCCESS" if success else "ERROR")
    
    @pyqtSlot(str, str)
    def _on_hardware_log(self, message: str, level: str):
        """Handle log messages from HardwareThread."""
        self.log_event(message, level)
    
    @pyqtSlot(str)
    def _on_hardware_error(self, error_msg: str):
        """Handle error messages from HardwareThread."""
        self.log_event(f"Hardware error: {error_msg}", "ERROR")

    # ── Pendant signal handlers ───────────────────────────────
    def _on_pendant_status(self, connected: bool, port: str = ""):
        """Handle pendant connection / disconnection signals."""
        self.pendant_connected = connected
        if connected:
            self.log_event(f"Pendant connected on {port}", "INFO")
            if hasattr(self, 'pendant_status_label'):
                self.pendant_status_label.setText(f"Pendant: {port}")
                self.pendant_status_label.setStyleSheet(
                    "color: #00cc00; font-weight: bold;")
        else:
            self.log_event("Pendant disconnected", "WARNING")
            if hasattr(self, 'pendant_status_label'):
                self.pendant_status_label.setText("Pendant: Not Connected")
                self.pendant_status_label.setStyleSheet(
                    "color: #888888; font-weight: bold;")

    def _on_pendant_command(self, command: str):
        """Parse the 21-char pendant command string.

        Character layout (each '0' or '1'):
          AXIS JOG (context-aware — Joint tab → joints, Cartesian tab → TCP):
            0  Axis1+  1  Axis1-    (J1 / X)
            2  Axis2+  3  Axis2-    (J2 / Y)
            4  Axis3+  5  Axis3-    (J3 / Z)
            6  Axis4+  7  Axis4-    (J4 / Yaw)
            8  Axis5+  9  Axis5-    (J5 / Pitch)
           10  Axis6+ 11  Axis6-    (J6 / Roll)

          ACTIONS (edge-triggered — fire once on 0→1 transition):
           12  Weld On/Off toggle
           13  Add current position to program
           14  Delete selected position from program
           15  Path type toggle (LINEAR → CURVE → P2P)
           16  Dry run
           17  Start / Play
           18  Abort
           19  Row back  (select previous row)
           20  Row forward (select next row)
        """
        prev = self._pendant_prev
        self._pendant_prev = command

        def _rising(idx: int) -> bool:
            """True on 0→1 transition (edge-detect)."""
            return (
                idx < len(command) and command[idx] == '1'
                and (idx >= len(prev) or prev[idx] == '0')
            )

        # ── AXIS JOG (chars 0-11) ─────────────────────────────
        jog_or_teach = (
            self.button_states.get('jog')
            or self.button_states.get('teach')
        )
        if jog_or_teach:
            is_joint = (
                hasattr(self, 'jog_tabs')
                and self.jog_tabs.currentIndex() == 0
            )

            if is_joint:
                # Joint space — simultaneous movement allowed
                joint_map = [
                    ('joint_0', +1), ('joint_0', -1),
                    ('joint_1', +1), ('joint_1', -1),
                    ('joint_2', +1), ('joint_2', -1),
                    ('joint_3', +1), ('joint_3', -1),
                    ('joint_4', +1), ('joint_4', -1),
                    ('joint_5', +1), ('joint_5', -1),
                ]
                for idx, (axis_id, direction) in enumerate(joint_map):
                    if idx < len(command) and command[idx] == '1':
                        self.execute_jog(axis_id, direction)
                self._pendant_cart_lock = None
            else:
                # Cartesian — single axis only, sticky to axis in motion
                cart_map = [
                    ('cart_x',  +1), ('cart_x',  -1),
                    ('cart_y',  +1), ('cart_y',  -1),
                    ('cart_z',  +1), ('cart_z',  -1),
                    ('cart_rz', +1), ('cart_rz', -1),
                    ('cart_ry', +1), ('cart_ry', -1),
                    ('cart_rx', +1), ('cart_rx', -1),
                ]
                # Collect all pressed axes this frame
                pressed = [
                    (axis_id, direction)
                    for idx, (axis_id, direction) in enumerate(cart_map)
                    if idx < len(command) and command[idx] == '1'
                ]
                if not pressed:
                    self._pendant_cart_lock = None
                else:
                    # If locked axis is still pressed, keep it
                    if self._pendant_cart_lock is not None:
                        locked = [
                            (a, d) for a, d in pressed
                            if a == self._pendant_cart_lock
                        ]
                        if locked:
                            self.execute_jog(*locked[0])
                            return
                    # Lock to the first pressed axis
                    self._pendant_cart_lock = pressed[0][0]
                    self.execute_jog(*pressed[0])
                    return  # only one axis executed

        # ── EDGE-TRIGGERED ACTIONS (chars 12-20) ──────────────
        # 12 — Weld toggle
        if _rising(12):
            self.weld_toggle_btn.setChecked(not self.weld_toggle_btn.isChecked())
            self.toggle_weld_state()

        # 13 — Add current position (requires Teach mode)
        if _rising(13) and self.button_states.get('teach'):
            self.add_current_position_to_program()

        # 14 — Delete selected position (requires Teach mode)
        if _rising(14) and self.button_states.get('teach'):
            self.delete_selected_row()

        # 15 — Path type toggle (LINEAR → CURVE → P2P)
        if _rising(15):
            motion_cycle = ["LINEAR", "CURVE", "P2P"]
            try:
                idx = motion_cycle.index(self.current_motion_type)
            except ValueError:
                idx = 0
            new_type = motion_cycle[(idx + 1) % len(motion_cycle)]
            self.set_motion_type(new_type)

        # 16 — Dry run
        if _rising(16):
            self.dry_run_program()

        # 17 — Start / Play
        if _rising(17):
            self.play_program()

        # 18 — Abort
        if _rising(18):
            self.handle_abort()

        # 19 — Row back (select previous row in program table)
        if _rising(19):
            sel = self.program_table.selectionModel()
            if sel is None:
                return
            rows = sel.selectedRows()
            current = rows[0].row() if rows else 0
            new_row = max(0, current - 1)
            self.program_table.selectRow(new_row)

        # 20 — Row forward (select next row in program table)
        if _rising(20):
            sel = self.program_table.selectionModel()
            if sel is None:
                return
            rows = sel.selectedRows()
            current = rows[0].row() if rows else -1
            max_row = self.program_model.rowCount() - 1
            new_row = min(max_row, current + 1) if max_row >= 0 else 0
            self.program_table.selectRow(new_row)

    def _on_pendant_log(self, message: str):
        """Forward pendant thread log messages to the event log."""
        self.log_event(message, "INFO")
    # ──────────────────────────────────────────────────────────

    def _log_throttle_key(self, message: str, event_type: str) -> Optional[str]:
        """Return a stable key for repetitive low-value log spam."""
        msg = str(message)
        if event_type == "DEBUG":
            return f"DEBUG::{msg.split(':', 1)[0]}"
        for prefix in self._log_prefix_throttle_seconds:
            if msg.startswith(prefix):
                return prefix
        return None

    def _log_rate_limit_seconds(self, message: str, event_type: str) -> float:
        """Return the per-key log suppression window in seconds."""
        if event_type == "DEBUG":
            return 10.0
        msg = str(message)
        for prefix, seconds in self._log_prefix_throttle_seconds.items():
            if msg.startswith(prefix):
                return float(seconds)
        return 0.0

    def _should_emit_log(self, message: str, event_type: str) -> tuple[bool, str]:
        """Rate-limit repetitive background logs and summarize suppressed repeats."""
        window_s = self._log_rate_limit_seconds(message, event_type)
        if window_s <= 0.0:
            return True, message

        key = self._log_throttle_key(message, event_type)
        if key is None:
            return True, message

        now = time.monotonic()
        state = self._log_throttle_state.get(key)
        if state is not None and (now - state['last_ts']) < window_s:
            state['suppressed'] += 1
            self._log_throttle_state[key] = state
            return False, message

        suppressed = int(state['suppressed']) if state is not None else 0
        self._log_throttle_state[key] = {'last_ts': now, 'suppressed': 0}
        if suppressed > 0:
            return True, f"{message} (suppressed {suppressed} similar messages)"
        return True, message
        
    def update_ui_components(self):
        """Update UI component states based on current button states"""
        # Rule: If Jog or Teach is not enabled, disable Jog panel
        jog_or_teach_enabled = self.button_states['jog'] or self.button_states['teach']
        self.jog_panel.setEnabled(jog_or_teach_enabled)
        
        # Rule: If Teach is disabled, disable table data manipulator buttons
        teach_enabled = self.button_states['teach']
        
        # Position and data manipulation buttons (require Teach mode)
        self.add_row_btn.setEnabled(teach_enabled)
        self.delete_row_btn.setEnabled(teach_enabled)
        self.add_timer_btn.setEnabled(teach_enabled)
        self.add_trigger_btn.setEnabled(teach_enabled)
        self.add_home_btn.setEnabled(teach_enabled)
        
        # Execution buttons (require connection but not necessarily Teach)
        connected = self.button_states['connect']
        self.generate_path_btn.setEnabled(connected)
        self.play_btn.setEnabled(connected)
        self.dry_run_btn.setEnabled(connected)
        
        # Position control buttons (require connection)
        self.move_to_position_button.setEnabled(connected)
        self.record_dropdown.setEnabled(connected)
        self.set_record_button.setEnabled(connected)
        self.set_home_button.setEnabled(connected)
        self.go_record_button.setEnabled(connected)
        self.go_home_button.setEnabled(connected)
        
        # Also disable other table manipulation buttons when teach is off
        if hasattr(self, 'save_btn'):
            # Save/Save As/Load/New can be used anytime when connected
            connected = self.button_states['connect']
            
            # Save button is only enabled when there's a current file loaded
            file_loaded = connected and self.current_program_file != "Untitled Program"
            self.save_btn.setEnabled(file_loaded)
            
            # Save As is always available when connected (can save new or existing)
            self.save_as_btn.setEnabled(connected)
            
            # Load and New are always available when connected
            self.load_btn.setEnabled(connected)
            self.new_btn.setEnabled(connected)
        
    def setup_weld_panel(self):
        """Set up the welding controls panel"""
        layout = QVBoxLayout(self.weld_panel)
        layout.setSpacing(3)  # Reduce spacing between groups
        layout.setContentsMargins(5, 5, 5, 5)  # Reduce margins
        
        # Current control (0-100)
        current_group = QGroupBox("Current Control")
        current_layout = QGridLayout(current_group)
        current_layout.setSpacing(3)  # Compact layout
        
        current_layout.addWidget(QLabel("Current:"), 0, 0)
        self.current_slider = QSlider(Qt.Orientation.Horizontal)
        self.current_slider.setRange(0, 100)
        self.current_slider.setValue(50)
        self.current_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.current_slider.setTickInterval(10)
        current_layout.addWidget(self.current_slider, 0, 1)
        
        self.current_spinbox = QSpinBox()
        self.current_spinbox.setRange(0, 100)
        self.current_spinbox.setValue(50)
        self.current_spinbox.setSuffix("%")
        current_layout.addWidget(self.current_spinbox, 0, 2)
        
        # Connect slider and spinbox
        self.current_slider.valueChanged.connect(self.current_spinbox.setValue)
        self.current_spinbox.valueChanged.connect(self.current_slider.setValue)
        
        layout.addWidget(current_group)
        
        # Wire Speed (Voltage) control (0-100)
        wire_group = QGroupBox("Wire Speed (Voltage)")
        wire_layout = QGridLayout(wire_group)
        wire_layout.setSpacing(3)  # Compact layout
        
        wire_layout.addWidget(QLabel("Wire Speed:"), 0, 0)
        self.wire_slider = QSlider(Qt.Orientation.Horizontal)
        self.wire_slider.setRange(0, 100)
        self.wire_slider.setValue(30)
        self.wire_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.wire_slider.setTickInterval(10)
        wire_layout.addWidget(self.wire_slider, 0, 1)
        
        self.wire_spinbox = QSpinBox()
        self.wire_spinbox.setRange(0, 100)
        self.wire_spinbox.setValue(30)
        self.wire_spinbox.setSuffix("%")
        wire_layout.addWidget(self.wire_spinbox, 0, 2)
        
        # Connect slider and spinbox
        self.wire_slider.valueChanged.connect(self.wire_spinbox.setValue)
        self.wire_spinbox.valueChanged.connect(self.wire_slider.setValue)
        
        layout.addWidget(wire_group)
        
        # Weld on/off control
        weld_control_group = QGroupBox("Weld Control")
        weld_control_layout = QHBoxLayout(weld_control_group)
        weld_control_layout.setSpacing(5)
        weld_control_layout.setContentsMargins(5, 5, 5, 5)
        
        self.weld_toggle_btn = QPushButton("Weld OFF")
        self.weld_toggle_btn.setCheckable(True)
        self.weld_toggle_btn.setChecked(self.welding_enabled)
        self.weld_toggle_btn.clicked.connect(self.toggle_weld_state)
        
        # Style the weld button
        weld_button_style = """
        QPushButton {
            padding: 8px 16px;
            font-weight: bold;
            border: 2px solid #ccc;
            border-radius: 5px;
            background-color: #f0f0f0;
            color: black;
            min-width: 100px;
        }
        QPushButton:checked {
            background-color: #FFA500;
            color: white;
            border-color: #FF8C00;
        }
        QPushButton:hover {
            background-color: #e0e0e0;
        }
        QPushButton:checked:hover {
            background-color: #FFB84D;
        }
        """
        self.weld_toggle_btn.setStyleSheet(weld_button_style)
        
        weld_control_layout.addWidget(self.weld_toggle_btn)
        weld_control_layout.addStretch()
        layout.addWidget(weld_control_group)
        
        # Combine smaller controls in a compact grid
        compact_group = QGroupBox("Settings")
        compact_layout = QGridLayout(compact_group)
        compact_layout.setSpacing(3)
        compact_layout.setContentsMargins(5, 5, 5, 5)
        
        # Weaving type selection (row 0)
        compact_layout.addWidget(QLabel("Weaving:"), 0, 0)
        self.weaving_combo = QComboBox()
        self.weaving_combo.addItems(["Linear", "Circle", "Sine", "Triangular", "Square", "Zigzag"])
        compact_layout.addWidget(self.weaving_combo, 0, 1)
        
        # Weave amplitude (row 1)
        compact_layout.addWidget(QLabel("Amp (mm):"), 1, 0)
        self.weave_amp_spinbox = QSpinBox()
        self.weave_amp_spinbox.setRange(0, 50)
        self.weave_amp_spinbox.setValue(3)
        self.weave_amp_spinbox.setMinimumWidth(80)
        self.weave_amp_spinbox.setToolTip("Weave half-width — lateral displacement from path centreline")
        compact_layout.addWidget(self.weave_amp_spinbox, 1, 1)
        
        # Weave frequency (row 2)
        compact_layout.addWidget(QLabel("Freq (Hz):"), 2, 0)
        from PyQt6.QtWidgets import QDoubleSpinBox
        self.weave_freq_spinbox = QDoubleSpinBox()
        self.weave_freq_spinbox.setRange(0.1, 20.0)
        self.weave_freq_spinbox.setValue(2.5)
        self.weave_freq_spinbox.setSingleStep(0.1)
        self.weave_freq_spinbox.setDecimals(1)
        self.weave_freq_spinbox.setMinimumWidth(80)
        self.weave_freq_spinbox.setToolTip("Weave oscillation frequency")
        compact_layout.addWidget(self.weave_freq_spinbox, 2, 1)
        
        # Weave dwell (row 3)
        compact_layout.addWidget(QLabel("Dwell (s):"), 3, 0)
        self.weave_dwell_spinbox = QDoubleSpinBox()
        self.weave_dwell_spinbox.setRange(0.0, 5.0)
        self.weave_dwell_spinbox.setValue(0.0)
        self.weave_dwell_spinbox.setSingleStep(0.1)
        self.weave_dwell_spinbox.setDecimals(1)
        self.weave_dwell_spinbox.setMinimumWidth(80)
        self.weave_dwell_spinbox.setToolTip("Pause time at each weave extreme for edge fusion")
        compact_layout.addWidget(self.weave_dwell_spinbox, 3, 1)
        
        # Timer control (row 4)
        compact_layout.addWidget(QLabel("Timer:"), 4, 0)
        self.timer_spinbox = QSpinBox()
        self.timer_spinbox.setRange(0, 999)
        self.timer_spinbox.setValue(0)
        self.timer_spinbox.setSuffix(" sec")
        compact_layout.addWidget(self.timer_spinbox, 4, 1)
        
        # Sensing trigger selection (row 5)
        compact_layout.addWidget(QLabel("Trigger:"), 5, 0)
        self.sensing_combo = QComboBox()
        # Create list with "None" first, then 1-16
        sensing_items = ["None"] + [str(i) for i in range(1, 17)]
        self.sensing_combo.addItems(sensing_items)
        self.sensing_combo.setCurrentText("None")  # Set default to None
        compact_layout.addWidget(self.sensing_combo, 5, 1)
        
        layout.addWidget(compact_group)
        
        # Remove stretch to make panel more compact
        
    def setup_console_panel(self):
        """Set up the event console for real-time logging"""
        layout = QVBoxLayout(self.console_panel)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)
        
        # Console header with clear button
        header_layout = QHBoxLayout()
        console_label = QLabel("Event Log:")
        console_label.setStyleSheet("font-weight: bold;")
        header_layout.addWidget(console_label)
        header_layout.addStretch()
        
        self.clear_console_btn = QPushButton("Clear")
        self.clear_console_btn.setMaximumWidth(60)
        self.clear_console_btn.clicked.connect(self.clear_console)
        header_layout.addWidget(self.clear_console_btn)
        
        layout.addLayout(header_layout)
        
        # Console text area
        self.console_text = QTextEdit()
        self.console_text.setReadOnly(True)
        self.console_text.setFont(QFont("Monaco", 9))  # Monospace font
        
        # Set console styling
        self.console_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #555;
                border-radius: 3px;
            }
        """)
        
        layout.addWidget(self.console_text)
        
    def log_event(self, message: str, event_type: str = "INFO"):
        """Log an event to the console with timestamp and type (batched for performance).
        Also appends a plain-text line to the in-memory buffer that gets
        flushed to a log file when the application closes."""
        should_emit, message = self._should_emit_log(str(message), str(event_type))
        if not should_emit:
            return

        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Color coding based on event type
        color_map = {
            "INFO": "#ffffff",       # White
            "SUCCESS": "#00ff00",    # Green
            "WARNING": "#ffaa00",    # Orange
            "ERROR": "#ff4444",      # Red
            "JOG": "#00aaff",        # Blue
            "WELD": "#ff8800",       # Orange
            "PROGRAM": "#aa88ff",    # Purple
            "CORRECTION": "#00ddbb", # Teal
            "DEBUG": "#888888",      # Gray
        }
        
        color = color_map.get(event_type, "#ffffff")
        
        # Format the message with HTML for color
        formatted_message = f'<span style="color: {color}">[{timestamp}] [{event_type}] {message}</span>'
        
        # Batch console appends to reduce repaint frequency
        self._console_batch_buffer.append(formatted_message)
        
        # Buffer plain-text line for file output
        date_stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self._log_buffer.append(f"[{date_stamp}] [{event_type}] {message}")
        
        # Start batch timer if not already running
        if not self._console_batch_timer.isActive():
            self._console_batch_timer.start(self._console_batch_timeout_ms)
    
    def _flush_console_batch(self):
        """Flush batched console messages to UI in one operation."""
        if not self._console_batch_buffer:
            return
        
        # Append all batched messages at once (single repaint)
        for formatted_message in self._console_batch_buffer:
            self.console_text.append(formatted_message)
        
        self._console_batch_buffer.clear()
        
        # Limit console lines to prevent excessive memory usage
        cursor = self.console_text.textCursor()
        cursor.movePosition(cursor.MoveOperation.Start)
        if cursor.blockNumber() > 500:  # Keep last 500 lines
            cursor.select(cursor.SelectionType.BlockUnderCursor)
            cursor.removeSelectedText()
        
        # Auto-scroll to bottom
        scrollbar = self.console_text.verticalScrollBar()
        if scrollbar is not None:
            scrollbar.setValue(scrollbar.maximum())
        
    def clear_console(self):
        """Clear the event console"""
        # Flush any pending batched messages first
        if self._console_batch_timer.isActive():
            self._console_batch_timer.stop()
            self._flush_console_batch()
        self.console_text.clear()
        self.log_event("Console cleared", "INFO")

    # ── Persistent logging helpers ──────────────────────────────────────────
    def _flush_log_to_file(self):
        """Write the accumulated log buffer to a timestamped file in logs/."""
        if not self._log_buffer:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(self.log_dir, f"session_{ts}.log")
        try:
            with open(log_path, 'w') as f:
                f.write("# Kinmatech Robotics \u2014 session log\n")
                f.write(f"# Started: {self._log_buffer[0].split(']')[0][1:]}\n")
                f.write(f"# Ended  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Entries: {len(self._log_buffer)}\n\n")
                for line in self._log_buffer:
                    f.write(line + '\n')
            print(f"Session log saved: {log_path}")
        except Exception as e:
            print(f"Failed to save session log: {e}")

    def _purge_old_logs(self, max_age_days: int = 30):
        """Delete log files older than *max_age_days*."""
        cutoff = datetime.now() - timedelta(days=max_age_days)
        removed = 0
        for path in glob.glob(os.path.join(self.log_dir, 'session_*.log')):
            try:
                mtime = datetime.fromtimestamp(os.path.getmtime(path))
                if mtime < cutoff:
                    os.remove(path)
                    removed += 1
            except Exception:
                pass
        if removed:
            print(f"Purged {removed} log file(s) older than {max_age_days} days")

    # ── Speed override ────────────────────────────────────────────────
    def _on_speed_override_changed(self, text: str):
        """Update the global speed override factor from the toolbar combo."""
        try:
            self.speed_override_factor = int(text.replace('%', '')) / 100.0
        except ValueError:
            self.speed_override_factor = 1.0
        # Also scale the 3D visualiser playback speed if playing
        viz = getattr(self, 'robot_visualizer', None)
        if viz is not None:
            viz.traj_speed_mult = self.speed_override_factor
            viz.speed_label.setText(f"{self.speed_override_factor:.1f}\u00d7")
            if viz.traj_playing and not viz.traj_paused:
                interval = max(1, int(viz.traj_interval_ms / viz.traj_speed_mult))
                viz.traj_timer.setInterval(interval)
        self.log_event(f"Speed override \u2192 {text}", "INFO")
        
    def initialize_ui_status(self):
        """Initialize UI status displays after startup"""
        if hasattr(self, 'update_home_button_status'):
            self.update_home_button_status()
        if hasattr(self, 'update_record_display'):
            self.update_record_display()
        
    def setup_record_dropdown(self):
        """Setup the record dropdown with 16 colored items"""
        # Define 16 distinct colors for the record positions
        colors = [
            "#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF", "#00FFFF", "#FFA500", "#800080",
            "#008000", "#FFC0CB", "#A52A2A", "#808080", "#000080", "#008080", "#800000", "#ADFF2F"
        ]
        
        # Add items with color indicators
        for i in range(1, 17):
            color = colors[i-1]
            # Add colored square indicator to the text
            self.record_dropdown.addItem(f"● Record {i}")
            
        # Set dropdown styling with color indicators
        dropdown_style = f"""
        QComboBox {{
            padding: 4px 8px;
            border: 2px solid #ccc;
            border-radius: 3px;
            background-color: #f0f0f0;
            min-width: 100px;
            font-weight: bold;
        }}
        QComboBox::drop-down {{
            border: none;
        }}
        QComboBox::down-arrow {{
            width: 12px;
            height: 12px;
        }}
        QComboBox QAbstractItemView {{
            selection-background-color: #4CAF50;
            background-color: white;
        }}
        """
        
        # Set individual item colors using delegate (more reliable approach)
        self.record_dropdown.setStyleSheet(dropdown_style)
        
        # Connect to update display when selection changes
        self.record_dropdown.currentIndexChanged.connect(self.update_record_display)
        
    def setup_program_table(self):
        """Set up the program table with empty model"""
        # Create empty program data
        empty_program_rows = create_empty_program_rows()
        
        # Create and set the table model
        self.program_model = ProgramTableModel(empty_program_rows)
        self.program_table.setModel(self.program_model)
        
        # Configure table appearance
        self.program_table.setAlternatingRowColors(True)
        self.program_table.setSelectionBehavior(QTableView.SelectionBehavior.SelectRows)
        self.program_table.setSelectionMode(QTableView.SelectionMode.SingleSelection)
        
        # Configure column sizing for better readability
        header = self.program_table.horizontalHeader()
        header.setMinimumSectionSize(60)
        
        # Set specific column widths
        self.program_table.setColumnWidth(0, 40)   # Idx
        self.program_table.setColumnWidth(1, 80)  # Motion Type (increased from 60)
        self.program_table.setColumnWidth(2, 70)   # X
        self.program_table.setColumnWidth(3, 70)   # Y
        self.program_table.setColumnWidth(4, 70)   # Z
        self.program_table.setColumnWidth(5, 70)   # RX
        self.program_table.setColumnWidth(6, 70)   # RY
        self.program_table.setColumnWidth(7, 70)   # RZ
        self.program_table.setColumnWidth(8, 60)   # Speed
        self.program_table.setColumnWidth(9, 60)   # WeldOn
        self.program_table.setColumnWidth(10, 60)  # Power
        self.program_table.setColumnWidth(11, 70)  # WireFeed
        self.program_table.setColumnWidth(12, 80)  # Weaving
        self.program_table.setColumnWidth(13, 60)  # Amp(mm)
        self.program_table.setColumnWidth(14, 60)  # Freq(Hz)
        self.program_table.setColumnWidth(15, 60)  # Dwell(s)
        self.program_table.setColumnWidth(16, 60)  # Timer
        self.program_table.setColumnWidth(17, 60)  # Trigger
        
        # Make comment column stretch to fill remaining space
        header.setStretchLastSection(True)
    
    def get_current_weld_params(self) -> WeldParams:
        """Get current welding parameters from weld panel for program entries"""
        return WeldParams(
            on=self.welding_enabled,  # Weld parameter setting (not actual torch state)
            power=float(self.current_spinbox.value()),
            wire_feed=float(self.wire_spinbox.value()),
            gas_pre=0.5,  # Default values
            gas_post=1.0,
            weaving_type=self.weaving_combo.currentText(),
            timer=self.timer_spinbox.value(),
            sensing_trigger=self.sensing_combo.currentText(),
            weave_amplitude=float(self.weave_amp_spinbox.value()),
            weave_frequency=self.weave_freq_spinbox.value(),
            weave_dwell=self.weave_dwell_spinbox.value(),
        )
    
    def add_current_position_to_program(self):
        """Add current robot position and weld settings to the program table"""
        # Get current position data
        current_pose = Pose(
            x=self.tcp_pose[0],
            y=self.tcp_pose[1], 
            z=self.tcp_pose[2],
            rx=self.tcp_pose[3],
            ry=self.tcp_pose[4],
            rz=self.tcp_pose[5]
        )
        
        # Get current welding parameters
        weld_params = self.get_current_weld_params()
        
        # Create new program row
        new_row_idx = len(self.program_model.program_data) + 1
        new_row = ProgramRow(
            idx=new_row_idx,
            type=self.current_motion_type,  # Use selected motion type
            pose=current_pose,
            joints_deg=self.current_joints.copy(),
            speed=self.movement_speed,  # Use current speed from slider
            accel=100.0,  # Default acceleration
            blend=2.0,   # Default blend
            weld=weld_params,
            comment=f"Position {new_row_idx}"
        )
        
        # Add to model
        self.program_model.add_row(new_row)
        self.program_modified = True
        self.update_program_title()
        
        # Log the position capture
        pos_str = f"X:{current_pose.x:.1f}, Y:{current_pose.y:.1f}, Z:{current_pose.z:.1f}"
        self.log_event(f"Position {new_row_idx} captured: {pos_str} (Motion: {self.current_motion_type})", "PROGRAM")
        
    def delete_selected_row(self):
        """Delete the currently selected row from the program"""
        sel = self.program_table.selectionModel()
        if sel is None:
            self.log_event("Program table not ready", "WARNING")
            return
        selection = sel.selectedRows()
        if selection:
            row_index = selection[0].row()
            self.program_model.remove_row(row_index)
            
            # Renumber remaining rows
            for i, row in enumerate(self.program_model._program_rows):
                row.idx = i + 1
            
            self.program_modified = True
            self.update_program_title()
            self.log_event(f"Deleted row {row_index + 1}, {len(self.program_model._program_rows)} rows remaining", "PROGRAM")
    
    def update_program_title(self):
        """Update the program title label with filename and modified status"""
        title = f"Program: {self.current_program_file}"
        if self.program_modified:
            title += " *"
        self.program_title_label.setText(title)
    
    def save_program(self):
        """Save current program to JSON file"""
        if self.current_program_file == "Untitled Program":
            # Show message that user should use Save As for new programs
            QMessageBox.information(
                self, "Save Program", 
                "This is a new program. Please use 'Save As' to specify a filename."
            )
            self.save_program_as()
        else:
            self.save_program_to_file(self.current_program_file)
    
    def save_program_as(self):
        """Save program with new filename"""
        filename, ok = QInputDialog.getText(
            self, 'Save Program As', 'Enter program name:',
            text=self.current_program_file.replace('Untitled Program', 'New Program')
        )
        if ok and filename:
            if not filename.endswith('.json'):
                filename += '.json'
            self.save_program_to_file(filename)
    
    def save_program_to_file(self, filename: str):
        """Save program data to specified file in positions directory structure"""
        try:
            # Remove .json extension to get program name for directory
            program_name = filename.replace('.json', '')
            
            # Create positions directory if it doesn't exist
            os.makedirs(self.positions_base_dir, exist_ok=True)
            
            # Create program-specific directory
            program_dir = os.path.join(self.positions_base_dir, program_name)
            os.makedirs(program_dir, exist_ok=True)
            
            # Full path for the JSON file
            full_path = os.path.join(program_dir, filename)
            
            program_data = {
                'filename': filename,
                'program_name': program_name,
                'rows': [row.to_dict() for row in self.program_model.program_data],
                'created': time.time()
            }
            
            with open(full_path, 'w') as f:
                json.dump(program_data, f, indent=2)
                
            self.current_program_file = filename
            self.program_modified = False
            self.update_program_title()
            self.log_event(f"Program '{program_name}' saved with {len(self.program_model.program_data)} positions", "PROGRAM")
            self.status_bar.showMessage(f"Program saved to {program_dir}/")
            
        except Exception as e:
            self.log_event(f"Failed to save program: {str(e)}", "ERROR")
            QMessageBox.critical(self, "Save Error", f"Could not save program: {str(e)}")
    
    def load_program(self):
        """Load program from JSON file in positions directory"""
        # Start browsing from positions directory if it exists
        start_dir = self.positions_base_dir if os.path.exists(self.positions_base_dir) else ''
        
        filename, _ = QFileDialog.getOpenFileName(
            self, 'Load Program', start_dir, 'JSON files (*.json)'
        )
        if filename:
            self.load_program_from_file(filename)
    
    def load_program_from_file(self, filename: str):
        """Load program data from specified file"""
        try:
            with open(filename, 'r') as f:
                program_data = json.load(f)
            
            # Load program rows
            rows = [ProgramRow.from_dict(row_data) for row_data in program_data.get('rows', [])]
            
            # Update model
            self.program_model.set_program_data(rows)
            
            # Update filename and status (use just the filename, not full path)
            self.current_program_file = os.path.basename(filename)
            self.program_modified = False
            self.update_program_title()
            self.update_ui_components()  # Update button states after loading
            
            # Log successful load
            program_name = program_data.get('program_name', 'Unknown')
            self.log_event(f"Program '{program_name}' loaded with {len(rows)} positions", "PROGRAM")
            
            # Show the program directory path in status
            program_dir = os.path.dirname(filename)
            self.status_bar.showMessage(f"Program loaded from {program_dir}/")
            
        except Exception as e:
            self.log_event(f"Failed to load program: {str(e)}", "ERROR")
            QMessageBox.critical(self, "Load Error", f"Could not load program: {str(e)}")
    
    def new_program(self):
        """Create a new empty program"""
        if self.program_modified:
            reply = QMessageBox.question(
                self, 'Unsaved Changes', 
                'Current program has unsaved changes. Create new program anyway?',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        
        # Clear program data
        self.program_model.set_program_data([])
        self.current_program_file = "Untitled Program"
        self.program_modified = False
        self.update_program_title()
        self.update_ui_components()  # Update button states
        
        self.log_event("New empty program created", "PROGRAM")
        self.status_bar.showMessage("New program created")
        
    def generate_path(self) -> bool:
        """Generate interpolated path from program points.

        Returns True if generation succeeded, False otherwise.
        """
        if not self.program_model.program_data:
            QMessageBox.information(
                self, "Generate Path", 
                "No program points available. Add positions to generate a path."
            )
            return False
        
        # ── Pre-validate: CURVE segments need ≥ 3 points ────────────
        motion_rows = [r for r in self.program_model.program_data
                       if r.type not in ('TIMER', 'TRIGGER', 'HOME') and r.pose is not None]
        if len(motion_rows) < 2:
            QMessageBox.warning(
                self, "Generate Path",
                "Need at least 2 motion waypoints to generate a path."
            )
            return False
        
        # Check consecutive CURVE counts
        curve_run = 0
        for row in motion_rows:
            if row.type == 'CURVE':
                curve_run += 1
            else:
                if 0 < curve_run < 3:
                    QMessageBox.warning(
                        self, "Generate Path",
                        f"CURVE segments require at least 3 consecutive CURVE "
                        f"waypoints to fit a spline, but found only {curve_run}.\n\n"
                        f"Either add more CURVE points or change the motion type "
                        f"to LINEAR or P2P."
                    )
                    return False
                curve_run = 0
        # Check trailing run
        if 0 < curve_run < 3:
            QMessageBox.warning(
                self, "Generate Path",
                f"CURVE segments require at least 3 consecutive CURVE "
                f"waypoints to fit a spline, but found only {curve_run}.\n\n"
                f"Either add more CURVE points or change the motion type "
                f"to LINEAR or P2P."
            )
            return False
        
        # Check if current program is saved
        if self.current_program_file == "Untitled Program" or self.program_modified:
            reply = QMessageBox.question(
                self, "Generate Path",
                "Program must be saved before generating path. Save now?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.Yes:
                self.save_program_as()
                if self.current_program_file == "Untitled Program":
                    return False  # User cancelled save
            else:
                return False
        
        try:
            # Get current program file path
            program_name = self.current_program_file.replace('.json', '')
            program_dir = os.path.join(self.positions_base_dir, program_name)
            program_file_path = os.path.join(program_dir, self.current_program_file)
            
            if not os.path.exists(program_file_path):
                QMessageBox.warning(
                    self, "Generate Path",
                    f"Program file not found: {program_file_path}"
                )
                return False
            
            # Log path generation start
            num_points = len(motion_rows)
            self.log_event(f"Starting path generation for {num_points} waypoints", "PROGRAM")
            self.status_bar.showMessage("Generating interpolated path...")
            
            # Generate path using path generator
            success = self.path_generator.generate_path_from_program(program_file_path)
            
            if success:
                output_file = self.path_generator.get_output_filename(program_file_path)
                gen_warnings = list(getattr(self.path_generator, 'last_generation_warnings', []))
                
                # Count output trajectory points
                traj_count = 0
                try:
                    with open(output_file, 'r') as f:
                        traj_count = sum(1 for line in f
                                         if line.strip() and not line.startswith('#'))
                except Exception:
                    pass
                
                self.log_event(
                    f"Path generated: {num_points} waypoints → {traj_count} servo points",
                    "SUCCESS"
                )
                for warning in gen_warnings:
                    self.log_event(warning, "WARNING")
                self.status_bar.showMessage(f"Path generated: {os.path.basename(output_file)}")
                
                warning_text = ""
                if gen_warnings:
                    warning_text = (
                        "\n\nWarnings:\n- " + "\n- ".join(gen_warnings[:3])
                    )
                    if len(gen_warnings) > 3:
                        warning_text += f"\n- ... and {len(gen_warnings) - 3} more"

                QMessageBox.information(
                    self, "Generate Path",
                    f"Path generation successful!\n\n"
                    f"Taught waypoints:  {num_points}\n"
                    f"Trajectory points: {traj_count}\n"
                    f"Output: {os.path.basename(output_file)}"
                    f"{warning_text}"
                )

                # Auto-load generated trajectory into 3D visualizer
                if self.robot_visualizer is not None:
                    try:
                        self.robot_visualizer.load_trajectory_file(output_file)
                    except Exception:
                        pass  # non-critical: visualizer update is best-effort
                return True
            else:
                self.log_event("Path generation failed", "ERROR")
                self.status_bar.showMessage("Path generation failed")
                QMessageBox.critical(
                    self, "Generate Path",
                    "Path generation failed. Check console for details."
                )
                return False
                
        except Exception as e:
            error_msg = f"Path generation error: {str(e)}"
            self.log_event(error_msg, "ERROR")
            self.status_bar.showMessage("Path generation failed")
            QMessageBox.critical(self, "Generate Path", error_msg)
            return False
    
    # ======================== TRAJECTORY ANALYSIS ==========================

    def _analyse_trajectory(self, file_path: str) -> dict:
        """Internally analyse a trajectory file for pose quality issues.

        Checks performed:
          1. Joint velocity spikes  – IK solution flips between frames
          2. Wrist singularity      – J5 within ±5° of 0°
          3. Joint-limit proximity  – any joint within 10° of ±180°
          4. Config consistency     – sudden elbow/shoulder sign flip

        Returns a dict:
            {
              'ok':       bool,       # True if no critical issues
              'warnings': [str, …],   # human-readable per-issue strings
              'critical': [str, …],   # showstoppers (solution flips, etc.)
              'stats':    dict,        # summary numbers
            }
        """
        if file_path.lower().endswith('.ksm'):
            ok, msg = validate_ksm_metadata(read_ksm_metadata(file_path))
            if not ok:
                return {
                    'ok': False,
                    'warnings': [],
                    'critical': [f'KSM verification failed: {msg}'],
                    'stats': {}
                }

        # ── Parse joint data from file ──
        joints = []
        cart_positions = []
        try:
            with open(file_path, 'r') as fh:
                for line in fh:
                    if line.startswith('#') or '--- Timer' in line or '--- Trigger' in line:
                        continue
                    parts = [x.strip() for x in line.split(',')]
                    if len(parts) < 6:
                        continue
                    try:
                        joints.append([float(parts[k]) for k in range(6)])
                        if len(parts) >= 10:
                            cart_positions.append([float(parts[7]), float(parts[8]), float(parts[9])])
                    except (ValueError, IndexError):
                        continue
        except Exception as e:
            return {'ok': False, 'warnings': [], 'critical': [f'Cannot read file: {e}'], 'stats': {}}

        if len(joints) < 2:
            return {'ok': True, 'warnings': [], 'critical': [], 'stats': {'frames': len(joints)}}

        ja = np.array(joints)            # (N, 6)
        n_frames = len(ja)

        warnings = []
        critical = []

        # ── Thresholds ──
        JOINT_LIMIT     = 180.0
        LIMIT_MARGIN    = 10.0           # warn if within this of ±180°
        SINGULARITY_DEG = 5.0            # J5 within ±5° of 0°
        VEL_SPIKE_THRESH = 15.0          # ° per frame — a likely IK flip
        # For 30 fps at 200 mm/s with ~1 mm steps, normal Δθ < ~2°/frame

        # ── 1. Joint velocities (frame-to-frame Δθ) ──
        # Use shortest-angle deltas so wrap across ±180° does not look like an IK flip.
        dj = np.diff(ja, axis=0)         # (N-1, 6)
        dj_wrapped = ((dj + 180.0) % 360.0) - 180.0
        dj_abs = np.abs(dj_wrapped)
        max_vel = np.max(dj_abs, axis=0)           # per-joint max
        max_vel_frame = np.argmax(dj_abs, axis=0)  # frame indices

        flip_frames = []
        for frame_idx in range(dj_abs.shape[0]):
            for jj in range(6):
                if dj_abs[frame_idx, jj] > VEL_SPIKE_THRESH:
                    flip_frames.append((frame_idx + 1, jj + 1, dj_abs[frame_idx, jj]))

        if flip_frames:
            # Group by frame to keep messages compact
            seen_frames = set()
            for fi, ji, dv in sorted(flip_frames, key=lambda x: -x[2]):
                if fi in seen_frames:
                    continue
                seen_frames.add(fi)
                msg = (f"Frame {fi}: J{ji} jumps {dv:.1f}° in one step "
                       f"(likely IK solution flip)")
                critical.append(msg)
                if len(seen_frames) >= 10:   # cap message count
                    remaining = len(set(f for f, _, _ in flip_frames)) - 10
                    if remaining > 0:
                        critical.append(f"… and {remaining} more velocity-spike frames")
                    break

        # ── 2. Wrist singularity (J5 ≈ 0°) ──
        j5 = ja[:, 4]
        sing_mask = np.abs(j5) < SINGULARITY_DEG
        n_sing = int(np.sum(sing_mask))
        if n_sing > 0:
            first_sing = int(np.argmax(sing_mask))
            pct = 100.0 * n_sing / n_frames
            msg = (f"Wrist singularity: J5 within ±{SINGULARITY_DEG}° of 0° "
                   f"for {n_sing} frames ({pct:.1f}%), first at frame {first_sing + 1} "
                   f"(J5={j5[first_sing]:.2f}°)")
            if pct > 5:
                critical.append(msg)
            else:
                warnings.append(msg)

        # ── 3. Joint-limit proximity ──
        for jj in range(6):
            col = ja[:, jj]
            near_pos = np.where(col > (JOINT_LIMIT - LIMIT_MARGIN))[0]
            near_neg = np.where(col < -(JOINT_LIMIT - LIMIT_MARGIN))[0]
            near = np.concatenate([near_pos, near_neg])
            if len(near) > 0:
                worst_idx = int(near[np.argmax(np.abs(col[near]))])
                worst_val = col[worst_idx]
                pct = 100.0 * len(near) / n_frames
                msg = (f"J{jj+1} near limit: {len(near)} frames ({pct:.1f}%) "
                       f"within {LIMIT_MARGIN}° of ±{JOINT_LIMIT}° "
                       f"(worst: {worst_val:.1f}° at frame {worst_idx + 1})")
                if np.abs(worst_val) > JOINT_LIMIT - 2.0:  # within 2°
                    critical.append(msg)
                else:
                    warnings.append(msg)

        # ── 4. Configuration consistency (elbow flip detection) ──
        # A sudden sign change in (J2 + J3) indicates an elbow config flip
        elbow_sum = ja[:, 1] + ja[:, 2]
        sign_changes = np.diff(np.sign(elbow_sum))
        elbow_flips = np.where(sign_changes != 0)[0]
        # Only flag if the change is large (>20°) — small oscillations near 0 are OK
        real_flips = []
        for idx in elbow_flips:
            delta = abs(elbow_sum[idx + 1] - elbow_sum[idx])
            if delta > 20:
                real_flips.append((idx + 1, delta))
        if real_flips:
            msg = (f"Elbow config flip detected at {len(real_flips)} frame(s): "
                   f"first at frame {real_flips[0][0] + 1} "
                   f"(Δ(J2+J3) = {real_flips[0][1]:.1f}°)")
            critical.append(msg)

        # ── 5. Stored Cartesian path vs FK from the joint frames ──
        cart_fk_mean_mm = None
        cart_fk_max_mm = None
        if len(cart_positions) == n_frames:
            fk_errors = []
            for idx, joint_row in enumerate(ja):
                fk_pose = solve_fk_gui(joint_row)
                err_mm = float(np.linalg.norm(np.array(fk_pose[:3]) - np.array(cart_positions[idx], dtype=float)))
                fk_errors.append(err_mm)

            if fk_errors:
                cart_fk_mean_mm = float(np.mean(fk_errors))
                cart_fk_max_mm = float(np.max(fk_errors))
                # A genuine kinematics mismatch causes high *mean* error
                # across all frames; a few outlier frames (singularity, IK
                # edge cases) can push max high while mean stays low.
                if cart_fk_mean_mm > 5.0:
                    critical.append(
                        "Stored Cartesian path does not match FK from joint frames "
                        f"(mean {cart_fk_mean_mm:.1f} mm, max {cart_fk_max_mm:.1f} mm). "
                        "Trajectory was generated with a different kinematics mapping; regenerate the path."
                    )
                elif cart_fk_max_mm > 20.0 or cart_fk_mean_mm > 0.5:
                    n_bad = sum(1 for e in fk_errors if e > 5.0)
                    warnings.append(
                        "FK/Cartesian outlier frames: "
                        f"mean {cart_fk_mean_mm:.1f} mm, max {cart_fk_max_mm:.1f} mm "
                        f"({n_bad} frame(s) > 5 mm, likely near singularity)."
                    )
                elif cart_fk_max_mm > 2.0:
                    warnings.append(
                        "Stored Cartesian path differs from FK from joint frames "
                        f"(mean {cart_fk_mean_mm:.1f} mm, max {cart_fk_max_mm:.1f} mm)."
                    )

        # ── Build stats summary ──
        stats = {
            'frames':            n_frames,
            'max_joint_vel':     [float(v) for v in max_vel],
            'max_vel_frames':    [int(f) + 1 for f in max_vel_frame],
            'j5_range':          (float(np.min(j5)), float(np.max(j5))),
            'singularity_frames': n_sing,
            'flip_frames':       len(set(f for f, _, _ in flip_frames)),
            'cartesian_fk_mean_mm': cart_fk_mean_mm,
            'cartesian_fk_max_mm': cart_fk_max_mm,
        }

        ok = len(critical) == 0
        return {'ok': ok, 'warnings': warnings, 'critical': critical, 'stats': stats}

    def _log_trajectory_analysis(self, result: dict, context: str = "Trajectory"):
        """Log analysis results to the console and optionally warn the user."""
        stats = result.get('stats', {})
        n = stats.get('frames', 0)

        if result['ok'] and not result['warnings']:
            self.log_event(
                f"{context} analysis: {n} frames — all checks passed ✓",
                "SUCCESS"
            )
            return

        # Log every warning
        for w in result['warnings']:
            self.log_event(f"⚠ {w}", "WARNING")

        # Log every critical issue
        for c in result['critical']:
            self.log_event(f"✖ {c}", "ERROR")

    def _find_trajectory_file(self):
        """Locate the generated joint-angles file for the current program."""
        if self.current_program_file == "Untitled Program":
            return None
        program_name = self.current_program_file.replace('.json', '')
        program_dir = os.path.join(self.positions_base_dir, program_name)
        ksm_file = os.path.join(program_dir, f"{program_name}_joint_angles.ksm")
        if os.path.exists(ksm_file):
            return ksm_file
        # Legacy fallback
        txt_file = os.path.join(program_dir, f"{program_name}_joint_angles.txt")
        if os.path.exists(txt_file):
            return txt_file
        return None

    def _start_trajectory_playback(self, traj_file, label="Playing"):
        """Load trajectory and P2P to start before playing."""
        if self.robot_visualizer is None:
            QMessageBox.warning(self, label, "3D Visualizer not available.")
            return

        # Save operator's weld settings; playback frames will drive weld state.
        self._preplay_welding_enabled = self.welding_enabled
        self._preplay_torch_active = self.torch_active
        self._traj_weld_sync_active = False

        # ── Pre-flight trajectory analysis (informational) ──
        analysis = self._analyse_trajectory(traj_file)
        self._log_trajectory_analysis(analysis, context=f"{label} pre-check")
        if not analysis.get('ok', True):
            critical = analysis.get('critical', [])
            mismatch_issue = next(
                (
                    msg for msg in critical
                    if "KSM verification failed" in msg
                    or "different kinematics mapping" in msg
                ),
                None,
            )
            if mismatch_issue is not None:
                self._restore_weld_state_after_playback()
                reply = QMessageBox.question(
                    self,
                    label,
                    f"{mismatch_issue}\n\n"
                    "The trajectory file was generated with an older "
                    "kinematics configuration.\n"
                    "Regenerate the path now?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                )
                if reply == QMessageBox.StandardButton.Yes:
                    ok = self.generate_path()
                    if ok:
                        # Verify the freshly written file before retrying
                        new_traj = self._find_trajectory_file()
                        if new_traj:
                            meta = read_ksm_metadata(new_traj)
                            valid, _ = validate_ksm_metadata(meta)
                            if valid:
                                self._start_trajectory_playback(new_traj, label)
                                return
                    # If we get here, regeneration didn't fix it
                    QMessageBox.warning(
                        self, label,
                        "Path regeneration did not resolve the kinematics "
                        "mismatch.\nPlease regenerate manually via "
                        "'Generate Path'.",
                    )
                return

        ok = self.robot_visualizer.load_trajectory_file(traj_file)
        if not ok:
            self._restore_weld_state_after_playback()
            QMessageBox.critical(self, label, "Failed to load trajectory file.")
            return

        if hasattr(self, 'status_tabs'):
            self.status_tabs.setCurrentIndex(1)  # Switch to 3D View

        # Move to the trajectory's first frame before playing
        if self.robot_visualizer.traj_joints is None or len(self.robot_visualizer.traj_joints) == 0:
            self._restore_weld_state_after_playback()
            QMessageBox.critical(self, label, "Trajectory loaded with no frames.")
            return

        first_joints = list(self.robot_visualizer.traj_joints[0])
        needs_move = any(
            abs(c - t) > 0.5
            for c, t in zip(self.current_joints, first_joints)
        )
        if needs_move:
            # Extract TCP from trajectory Cartesian data if available
            first_tcp = None
            viz = self.robot_visualizer
            if viz.traj_has_cartesian and len(viz.traj_cartesian) > 0:
                cd = viz.traj_cartesian[0]
                pos = cd['position']
                ori = cd['orientation']
                first_tcp = (pos[0], pos[1], pos[2], ori[0], ori[1], ori[2])
            mode = self.transition_mode
            self.log_event(f"{mode} to trajectory start position…", "JOG")
            self.start_transition_motion(
                first_joints, target_tcp=first_tcp,
                on_complete=lambda: self._begin_playback_after_p2p(traj_file, label)
            )
        else:
            self._begin_playback_after_p2p(traj_file, label)

    def _begin_playback_after_p2p(self, traj_file, label):
        """Called once P2P to start position is complete."""
        self._traj_weld_sync_active = True
        self.robot_visualizer.toggle_playback()
        self.log_event(f"{label}: {os.path.basename(traj_file)}", "PROGRAM")
        self.status_bar.showMessage(f"{label} trajectory…")

    def _set_weld_state_for_playback(self, weld_on: bool):
        """Apply weld state from trajectory frame without manual-toggle side effects."""
        self.welding_enabled = bool(weld_on)
        self.torch_active = bool(weld_on)

        # Keep button visuals synchronized without firing click handlers.
        if hasattr(self, 'weld_toggle_btn') and self.weld_toggle_btn is not None:
            prev = self.weld_toggle_btn.blockSignals(True)
            self.weld_toggle_btn.setChecked(self.welding_enabled)
            self.weld_toggle_btn.setText("Weld ON" if self.welding_enabled else "Weld OFF")
            self.weld_toggle_btn.blockSignals(prev)

        self.update_torch_status_display()

    def _restore_weld_state_after_playback(self):
        """Restore operator weld settings that were active before playback."""
        if self._preplay_welding_enabled is None:
            return

        self.welding_enabled = bool(self._preplay_welding_enabled)
        self.torch_active = bool(self._preplay_torch_active)

        if hasattr(self, 'weld_toggle_btn') and self.weld_toggle_btn is not None:
            prev = self.weld_toggle_btn.blockSignals(True)
            self.weld_toggle_btn.setChecked(self.welding_enabled)
            self.weld_toggle_btn.setText("Weld ON" if self.welding_enabled else "Weld OFF")
            self.weld_toggle_btn.blockSignals(prev)

        self.update_torch_status_display()
        self._traj_weld_sync_active = False
        self._preplay_welding_enabled = None
        self._preplay_torch_active = None

    def play_program(self):
        """Execute the robot program – plays trajectory in 3D viewer."""
        if not self.program_model.program_data:
            QMessageBox.information(
                self, "Play Program",
                "No program to execute. Add positions first."
            )
            return

        if not self.button_states['connect']:
            QMessageBox.warning(
                self, "Play Program",
                "Robot must be connected to execute program."
            )
            return

        traj_file = self._find_trajectory_file()
        if traj_file is None:
            QMessageBox.warning(
                self, "Play Program",
                "Trajectory file not found.\n\n"
                "Generate the path first using the 'Generate Path' button."
            )
            return

        self._start_trajectory_playback(traj_file, label="Playing program")

    def dry_run_program(self):
        """Simulate program execution in 3D viewer without welding."""
        if not self.program_model.program_data:
            QMessageBox.information(
                self, "Dry Run",
                "No program to simulate. Add positions first."
            )
            return

        traj_file = self._find_trajectory_file()
        if traj_file is None:
            QMessageBox.warning(
                self, "Dry Run",
                "Trajectory file not found.\n\n"
                "Generate the path first using the 'Generate Path' button."
            )
            return

        self._start_trajectory_playback(traj_file, label="Dry run")
    
    def add_timer_to_program(self):
        """Add timer command to program"""
        if not self.button_states['teach']:
            QMessageBox.warning(
                self, "Add Timer", 
                "Teach mode must be active to modify program."
            )
            return
            
        # Get timer value from weld panel
        timer_value = self.timer_spinbox.value()
        
        # Create new timer row
        new_row_idx = len(self.program_model.program_data) + 1
        timer_row = ProgramRow(
            idx=new_row_idx,
            type="TIMER",
            pose=None,  # No position for timer commands
            joints_deg=None,
            speed=0.0,
            accel=0.0,
            blend=0.0,
            weld=None,  # No welding for timer commands
            comment=f"Timer: {timer_value} seconds"
        )
        
        # Add to model
        self.program_model.add_row(timer_row)
        self.program_modified = True
        self.update_program_title()
        self.status_bar.showMessage(f"Added timer command: {timer_value} seconds")
    
    def add_trigger_to_program(self):
        """Add trigger command to program"""
        if not self.button_states['teach']:
            QMessageBox.warning(
                self, "Add Trigger", 
                "Teach mode must be active to modify program."
            )
            return
            
        # Get trigger value from weld panel
        trigger_value = self.sensing_combo.currentText()
        
        if trigger_value == "None":
            QMessageBox.information(
                self, "Add Trigger", 
                "Please select a trigger number (1-16) from the sensing trigger dropdown."
            )
            return
            
        # Create new trigger row
        new_row_idx = len(self.program_model.program_data) + 1
        trigger_row = ProgramRow(
            idx=new_row_idx,
            type="TRIGGER",
            pose=None,  # No position for trigger commands
            joints_deg=None,
            speed=0.0,
            accel=0.0,
            blend=0.0,
            weld=None,  # No welding for trigger commands
            comment=f"Trigger: {trigger_value}"
        )
        
        # Add to model
        self.program_model.add_row(trigger_row)
        self.program_modified = True
        self.update_program_title()
        self.status_bar.showMessage(f"Added trigger command: {trigger_value}")
        
    def add_home_to_program(self):
        """Add go-to-home command to program"""
        if not self.button_states['teach']:
            QMessageBox.warning(
                self, "Add Home", 
                "Teach mode must be active to modify program."
            )
            return
        
        # Create new home row
        new_row_idx = len(self.program_model.program_data) + 1
        home_row = ProgramRow(
            idx=new_row_idx,
            type="HOME",
            pose=None,  # No specific pose for home command
            joints_deg=None,
            speed=0.0,
            accel=0.0,
            blend=0.0,
            weld=None,  # No welding for home commands
            comment="Go to home position"
        )
        
        # Add to model
        self.program_model.add_row(home_row)
        self.program_modified = True
        self.update_program_title()
        self.status_bar.showMessage("Added go-to-home command")
        
    def move_to_selected_position(self):
        """Move robot to the selected position from the program table (P2P interpolated)"""
        sel = self.program_table.selectionModel()
        if sel is None:
            self.log_event("Program table selection unavailable", "WARNING")
            return
        selection = sel.selectedRows()
        
        if not selection:
            self.log_event("No position selected in program table", "WARNING")
            return
        
        row_index = selection[0].row()
        
        # Check if row index is valid
        if row_index >= len(self.program_model._program_rows) or row_index < 0:
            self.log_event("Invalid row selected", "ERROR")
            return
            
        # Get the program row directly from the model's data
        program_row = self.program_model._program_rows[row_index]
        
        # Check if this is a position row (TIMER/TRIGGER rows have no pose)
        if program_row.pose is None:
            self.log_event(f"Row {row_index + 1} is a {program_row.type} command, not a position", "WARNING")
            return
        
        target_pose = program_row.pose
        
        self.log_event(f"Moving to position: Row {row_index + 1}", "JOG")
        self.log_event(f"Target: X={target_pose.x:.1f}, Y={target_pose.y:.1f}, Z={target_pose.z:.1f}", "JOG")
        
        target_tcp = (target_pose.x, target_pose.y, target_pose.z,
                      target_pose.rx, target_pose.ry, target_pose.rz)
        
        # Use stored joint angles if available, otherwise use IK
        if program_row.joints_deg is not None:
            self.start_transition_motion(program_row.joints_deg, target_tcp)
        else:
            # Fallback: request IK for the target pose (no P2P – we don't have joints)
            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self.fk_thread.request_ik(target_tcp)
            self.log_event("Position move requested (via IK)", "SUCCESS")
        
    def set_record_position(self):
        """Set current position to the selected record with safety warning"""
        # Safety confirmation dialog
        reply = QMessageBox.warning(
            self, 
            "SAFETY WARNING", 
            "⚠️ WARNING: This operation has NO collision avoidance!\\n\\n"
            "The robot will move directly to this position when the record is triggered.\\n"
            "Ensure the path is clear and safe before proceeding.\\n\\n"
            "Do you want to continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        # Get selected record number
        record_num = self.record_dropdown.currentIndex() + 1
        
        # Store current position and orientation
        record_data = {
            'joints': self.current_joints.copy(),
            'unwrapped_wrist': self.unwrapped_wrist.copy(),
            'tcp_pose': self.tcp_pose,
            'timestamp': time.time()
        }
        
        self.record_positions[record_num] = record_data
        
        self.log_event(f"Record {record_num} set to current position", "SUCCESS")
        self.log_event(f"Position: X={self.tcp_pose[0]:.1f}, Y={self.tcp_pose[1]:.1f}, Z={self.tcp_pose[2]:.1f}", "JOG")
        
        # Save to jog_data.json
        self.save_jog_data()
        
    def set_new_home_position(self):
        """Set current position as the new home position"""
        # Store current position as home, including unwrapped wrist values for homing unwinding
        self.home_position = {
            'joints': self.current_joints.copy(),
            'unwrapped_wrist': self.unwrapped_wrist.copy(),  # Store absolute wrist rotations
            'tcp_pose': self.tcp_pose,
            'timestamp': time.time()
        }
        
        self.log_event("New home position set", "SUCCESS")
        self.log_event(f"Home: X={self.tcp_pose[0]:.1f}, Y={self.tcp_pose[1]:.1f}, Z={self.tcp_pose[2]:.1f}", "JOG")
        
        # Update Go Home button appearance to indicate home is set
        self.update_home_button_status()
        
        # Save to jog_data.json
        self.save_jog_data()
        
    def update_home_button_status(self):
        """Update Go Home button appearance based on whether home is set"""
        if self.home_position:
            # Home is set - make button green
            home_button_style = """
            QPushButton {
                padding: 8px 16px;
                font-weight: bold;
                border: 2px solid #4CAF50;
                border-radius: 5px;
                background-color: #4CAF50;
                color: white;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
                border-color: #aaaaaa;
            }
            """
            self.go_home_button.setStyleSheet(home_button_style)
            self.go_home_button.setStatusTip("Move robot to home position (HOME SET)")
        else:
            # Home not set - use default style
            button_style = """
            QPushButton {
                padding: 8px 16px;
                font-weight: bold;
                border: 2px solid #ccc;
                border-radius: 5px;
                background-color: #f0f0f0;
                color: black;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
                color: black;
            }
            """
            self.go_home_button.setStyleSheet(button_style)
            self.go_home_button.setStatusTip("Move robot to home position (HOME NOT SET)")
        
        # Save to jog_data.json
        self.save_jog_data()
        
    def go_to_record_position(self, record_num: int):
        """Move robot to a specific record position (1-16).
        
        First unwinds wrist rotations back to the stored unwrapped position,
        then moves to the record.
        """
        if record_num not in self.record_positions:
            self.log_event(f"Record {record_num} not set", "WARNING")
            return
        
        record_data = self.record_positions[record_num]
        
        self.log_event(f"Moving to Record {record_num} ({self.transition_mode})", "JOG")
        
        # Step 1: Unwind wrist to stored unwrapped position
        record_unwrapped = record_data.get('unwrapped_wrist', [0.0, 0.0, 0.0])
        unwind_joints = list(self.current_joints)
        unwind_joints[3:6] = [
            self._unwrap_angle_near(record_unwrapped[i], self.current_joints[3 + i])
            for i in range(3)
        ]
        
        # Step 2: If unwinding is needed, move to unwind position first
        wrist_delta = max(abs(unwind_joints[i] - self.current_joints[i]) for i in range(3, 6))
        if wrist_delta > 0.5:  # Only unwind if movement is significant
            self.log_event("Unwinding wrist rotations before moving to record...", "JOG")
            # Chain exactly on motion completion (no fixed timer), preventing
            # early hand-off that causes visible/sudden correction moves.
            self.start_transition_motion(
                unwind_joints,
                None,
                on_complete=lambda: self._do_goto_record_after_unwind(record_num),
            )
        else:
            # Wrist doesn't need unwinding, go straight to record
            target_tcp = record_data.get('tcp_pose', None)
            self.start_transition_motion(record_data['joints'], target_tcp)
    
    def _do_goto_record_after_unwind(self, record_num: int):
        """Called after wrist unwinding completes; moves to actual record position."""
        if record_num in self.record_positions:
            record_data = self.record_positions[record_num]
            target_tcp = record_data.get('tcp_pose', None)
            target_joints = [float(v) for v in record_data['joints']]
            # Keep wrist continuous relative to current pose to avoid a tiny
            # wrap-induced jump when switching from unwind leg to final leg.
            for i in range(3, 6):
                target_joints[i] = self._unwrap_angle_near(target_joints[i], self.current_joints[i])
            self.start_transition_motion(target_joints, target_tcp)
    
    def _update_record_position(self, record_num: int):
        """Update a record position with current state, including unwrapped wrist."""
        self.record_positions[record_num] = {
            'joints': self.current_joints.copy(),
            'unwrapped_wrist': self.unwrapped_wrist.copy(),
            'tcp_pose': self.tcp_pose,
            'timestamp': time.time()
        }
        
    def go_to_home_position(self):
        """Move robot to home position using current transition mode.
        
        First unwinds wrist rotations back to the stored unwrapped position,
        then moves to home.
        """
        if not self.home_position:
            self.log_event("Home position not set", "WARNING")
            return

        self.log_event(f"Moving to Home position ({self.transition_mode})", "JOG")

        # Step 1: Unwind wrist to stored unwrapped position
        home_unwrapped = self.home_position.get('unwrapped_wrist', [0.0, 0.0, 0.0])
        unwind_joints = list(self.current_joints)
        unwind_joints[3:6] = [
            self._unwrap_angle_near(home_unwrapped[i], self.current_joints[3 + i])
            for i in range(3)
        ]
        
        # Step 2: If unwinding is needed (wrist has rotated), first move to unwind position
        wrist_delta = max(abs(unwind_joints[i] - self.current_joints[i]) for i in range(3, 6))
        if wrist_delta > 0.5:  # Only unwind if movement is significant
            self.log_event("Unwinding wrist rotations before homing...", "JOG")
            # Chain exactly on motion completion (no fixed timer), preventing
            # early hand-off that causes visible/sudden correction moves.
            self.start_transition_motion(
                unwind_joints,
                None,
                on_complete=self._do_home_after_unwind,
            )
        else:
            # Wrist doesn't need unwinding, go straight to home
            target_tcp = self.home_position.get('tcp_pose', None)
            self.start_transition_motion(self.home_position['joints'], target_tcp)
    
    def _do_home_after_unwind(self):
        """Called after wrist unwinding completes; moves to actual home position."""
        if self.home_position:
            target_tcp = self.home_position.get('tcp_pose', None)
            target_joints = [float(v) for v in self.home_position['joints']]
            # Keep wrist continuous relative to current pose to avoid a tiny
            # wrap-induced jump when switching from unwind leg to final leg.
            for i in range(3, 6):
                target_joints[i] = self._unwrap_angle_near(target_joints[i], self.current_joints[i])
            self.start_transition_motion(target_joints, target_tcp)
    def go_to_selected_record(self):
        """Move robot to the currently selected record position"""
        record_num = self.record_dropdown.currentIndex() + 1
        
        # Safety confirmation dialog
        reply = QMessageBox.warning(
            self, 
            "SAFETY WARNING - GO TO RECORD", 
            f"⚠️ WARNING: Moving to Record {record_num} with NO collision avoidance!\n\n"
            "The robot will move directly to this position.\n"
            "Ensure the path is clear and safe before proceeding.\n\n"
            "Do you want to continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
            
        self.go_to_record_position(record_num)
        
    def update_record_display(self):
        """Update the record dropdown display with color coding"""
        colors = [
            "#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF", "#00FFFF", "#FFA500", "#800080",
            "#008000", "#FFC0CB", "#A52A2A", "#808080", "#000080", "#008080", "#800000", "#ADFF2F"
        ]
        
        current_index = self.record_dropdown.currentIndex()
        if 0 <= current_index < len(colors):
            color = colors[current_index]
            record_num = current_index + 1
            
            # Update dropdown style to show selected record color
            dropdown_style = f"""
            QComboBox {{
                padding: 4px 8px;
                border: 3px solid {color};
                border-radius: 3px;
                background-color: {color};
                color: white;
                min-width: 100px;
                font-weight: bold;
            }}
            QComboBox::drop-down {{
                border: none;
            }}
            QComboBox::down-arrow {{
                width: 12px;
                height: 12px;
            }}
            QComboBox QAbstractItemView {{
                selection-background-color: #4CAF50;
                background-color: white;
                color: black;
            }}
            """
            self.record_dropdown.setStyleSheet(dropdown_style)
            
            # Update status to show which record is selected
            is_set = record_num in self.record_positions
            status_text = f"Record {record_num} selected" + (" (SET)" if is_set else " (NOT SET)")
            self.status_bar.showMessage(status_text)
        
    def setup_jog_panel(self):
        """Set up the jog panel with joint and cartesian controls"""
        layout = QVBoxLayout(self.jog_panel)
        
        # Create tab widget for joint vs cartesian jogging
        self.jog_tabs = QTabWidget()
        layout.addWidget(self.jog_tabs)
        
        # Setup Joint jog tab
        self.setup_joint_jog_tab()
        
        # Setup Cartesian jog tab  
        self.setup_cartesian_jog_tab()
        
        # Add sync button
        sync_button = QPushButton("🔄 Sync Values")
        sync_button.setToolTip("Synchronize joint and cartesian values using forward kinematics")
        sync_button.clicked.connect(self.sync_jog_values)
        layout.addWidget(sync_button)
        
        # Add movement speed control
        speed_group = QGroupBox("Movement Speed")
        speed_layout = QHBoxLayout(speed_group)
        
        speed_layout.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 200)  # 1 to 200 mm/s or deg/s
        self.speed_slider.setValue(int(self.movement_speed))
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setTickInterval(25)
        self.speed_slider.setToolTip("Continuous hold speed (mm/s for XYZ, deg/s for joints/orientation)")
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_layout.addWidget(self.speed_slider)
        
        self.speed_input = QLineEdit(str(self.movement_speed))
        self.speed_input.setMinimumWidth(90)
        self.speed_input.setMaximumWidth(110)
        self.speed_input.textChanged.connect(self.on_speed_input_changed)
        speed_layout.addWidget(self.speed_input)
        
        speed_layout.addWidget(QLabel("mm/s"))
        layout.addWidget(speed_group)
        
        # Add motion type control
        motion_type_group = QGroupBox("Motion Type")
        motion_type_layout = QHBoxLayout(motion_type_group)
        
        self.linear_btn = QPushButton("Linear")
        self.linear_btn.setCheckable(True)
        self.linear_btn.setChecked(True)  # Default selection
        self.linear_btn.clicked.connect(lambda: self.set_motion_type("LINEAR"))
        motion_type_layout.addWidget(self.linear_btn)
        
        self.curve_btn = QPushButton("Curve")
        self.curve_btn.setCheckable(True)
        self.curve_btn.clicked.connect(lambda: self.set_motion_type("CURVE"))
        motion_type_layout.addWidget(self.curve_btn)

        self.p2p_btn = QPushButton("P2P")
        self.p2p_btn.setCheckable(True)
        self.p2p_btn.clicked.connect(lambda: self.set_motion_type("P2P"))
        motion_type_layout.addWidget(self.p2p_btn)
        
        # Style the motion type buttons
        motion_button_style = """
        QPushButton {
            padding: 6px 12px;
            font-weight: bold;
            border: 2px solid #ccc;
            border-radius: 5px;
            background-color: #f0f0f0;
            color: black;
            min-width: 60px;
        }
        QPushButton:hover {
            background-color: #e0e0e0;
        }
        QPushButton:checked {
            background-color: #4CAF50;
            color: white;
            border-color: #45a049;
        }
        """
        self.linear_btn.setStyleSheet(motion_button_style)
        self.curve_btn.setStyleSheet(motion_button_style)
        self.p2p_btn.setStyleSheet(motion_button_style)
        
        layout.addWidget(motion_type_group)
        
    def setup_joint_jog_tab(self):
        """Setup the joint jogging tab"""
        joint_widget = QWidget()
        joint_layout = QVBoxLayout(joint_widget)
        
        # Step size control
        step_group = QGroupBox("Fine Step Size (Tap)")
        step_layout = QHBoxLayout(step_group)
        
        step_layout.addWidget(QLabel("Degrees:"))
        self.joint_step_slider = QSlider(Qt.Orientation.Horizontal)
        self.joint_step_slider.setRange(1, 100)  # 0.1 to 10.0 degrees
        self.joint_step_slider.setValue(10)  # Default 1.0 degree
        self.joint_step_slider.valueChanged.connect(self.on_joint_step_changed)
        
        self.joint_step_input = QLineEdit("1.0")
        self.joint_step_input.setMinimumWidth(90)
        self.joint_step_input.setMaximumWidth(110)
        self.joint_step_input.setToolTip("Used for quick tap jogging only (deg/step)")
        self.joint_step_input.textChanged.connect(self.on_joint_step_input_changed)
        
        step_layout.addWidget(self.joint_step_slider)
        step_layout.addWidget(self.joint_step_input)
        joint_layout.addWidget(step_group)
        
        # Joint controls
        joints_group = QGroupBox("Joint Controls")
        joint_grid = QGridLayout(joints_group)
        
        self.joint_inputs = []
        
        for i in range(6):
            # Joint label
            label = QLabel(f"J{i+1}:")
            joint_grid.addWidget(label, i, 0)
            
            # Current position / Target input field
            target_input = QLineEdit("0.0")
            target_input.setMinimumWidth(110)
            target_input.setMaximumWidth(150)
            target_input.returnPressed.connect(lambda joint=i: self.go_to_joint_target(joint))
            joint_grid.addWidget(target_input, i, 1)
            self.joint_inputs.append(target_input)
            
            # Unit label
            unit_label = QLabel("°")
            joint_grid.addWidget(unit_label, i, 2)
            
            # Jog buttons
            minus_btn = QPushButton("-")
            plus_btn = QPushButton("+")
            minus_btn.setMaximumWidth(30)
            plus_btn.setMaximumWidth(30)
            
            # Setup long press for these buttons
            self.setup_jog_button(minus_btn, f"joint_{i}", -1)
            self.setup_jog_button(plus_btn, f"joint_{i}", 1)
            
            joint_grid.addWidget(minus_btn, i, 3)
            joint_grid.addWidget(plus_btn, i, 4)
            
        joint_layout.addWidget(joints_group)
        self.jog_tabs.addTab(joint_widget, "Joint Jog")
        
    def setup_cartesian_jog_tab(self):
        """Setup the cartesian jogging tab"""
        cartesian_widget = QWidget()
        cartesian_layout = QVBoxLayout(cartesian_widget)
        
        # Step size control
        step_group = QGroupBox("Fine Step Size (Tap)")
        step_layout = QHBoxLayout(step_group)
        
        step_layout.addWidget(QLabel("mm/deg:"))
        self.cartesian_step_slider = QSlider(Qt.Orientation.Horizontal)
        self.cartesian_step_slider.setRange(1, 100)  # 0.1 to 10.0 mm/deg
        self.cartesian_step_slider.setValue(10)  # Default 1.0 mm/deg
        self.cartesian_step_slider.valueChanged.connect(self.on_cartesian_step_changed)
        
        self.cartesian_step_input = QLineEdit("1.0")
        self.cartesian_step_input.setMinimumWidth(90)
        self.cartesian_step_input.setMaximumWidth(110)
        self.cartesian_step_input.setToolTip("Used for quick tap jogging only (mm/deg per step)")
        self.cartesian_step_input.textChanged.connect(self.on_cartesian_step_input_changed)
        
        step_layout.addWidget(self.cartesian_step_slider)
        step_layout.addWidget(self.cartesian_step_input)
        cartesian_layout.addWidget(step_group)
        
        # Cartesian controls
        cartesian_group = QGroupBox("Cartesian Controls")
        cartesian_grid = QGridLayout(cartesian_group)
        
        self.cartesian_inputs = {}
        
        # Position controls (X, Y, Z)
        pos_items = [('X', 'mm'), ('Y', 'mm'), ('Z', 'mm')]
        for i, (name, unit) in enumerate(pos_items):
            label = QLabel(f"{name}:")
            cartesian_grid.addWidget(label, i, 0)
            
            # Target input
            target_input = QLineEdit("0.0")
            target_input.setMinimumWidth(110)
            target_input.setMaximumWidth(150)
            target_input.returnPressed.connect(lambda axis=name.lower(): self.go_to_cartesian_target(axis))
            cartesian_grid.addWidget(target_input, i, 1)
            self.cartesian_inputs[name.lower()] = target_input
            
            # Unit label
            unit_label = QLabel(unit)
            cartesian_grid.addWidget(unit_label, i, 2)
            
            # Jog buttons
            minus_btn = QPushButton("-")
            plus_btn = QPushButton("+")
            minus_btn.setMaximumWidth(30)
            plus_btn.setMaximumWidth(30)
            
            self.setup_jog_button(minus_btn, f"cart_{name.lower()}", -1)
            self.setup_jog_button(plus_btn, f"cart_{name.lower()}", 1)
            
            cartesian_grid.addWidget(minus_btn, i, 3)
            cartesian_grid.addWidget(plus_btn, i, 4)
        
        # Orientation controls (RX, RY, RZ)
        rot_items = [('RX', '°'), ('RY', '°'), ('RZ', '°')]
        for i, (name, unit) in enumerate(rot_items, 3):
            label = QLabel(f"{name}:")
            cartesian_grid.addWidget(label, i, 0)
            
            # Target input
            target_input = QLineEdit("0.0")
            target_input.setMinimumWidth(110)
            target_input.setMaximumWidth(150)
            target_input.returnPressed.connect(lambda axis=name.lower(): self.go_to_cartesian_target(axis))
            cartesian_grid.addWidget(target_input, i, 1)
            self.cartesian_inputs[name.lower()] = target_input
            
            # Unit label
            unit_label = QLabel(unit)
            cartesian_grid.addWidget(unit_label, i, 2)
            
            # Jog buttons
            minus_btn = QPushButton("-")
            plus_btn = QPushButton("+")
            minus_btn.setMaximumWidth(30)
            plus_btn.setMaximumWidth(30)
            
            self.setup_jog_button(minus_btn, f"cart_{name.lower()}", -1)
            self.setup_jog_button(plus_btn, f"cart_{name.lower()}", 1)
            
            cartesian_grid.addWidget(minus_btn, i, 3)
            cartesian_grid.addWidget(plus_btn, i, 4)
        
        cartesian_layout.addWidget(cartesian_group)
        self.jog_tabs.addTab(cartesian_widget, "Cartesian Jog")
        
    def setup_status_panel(self):
        """Set up the status panel with robot state display and 3D visualization"""
        layout = QVBoxLayout(self.status_panel)
        
        # Create tabs for status information and 3D view
        status_tabs = QTabWidget()
        
        # Status Tab
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        # Joint Position display
        joint_group = QGroupBox("Joint Positions")
        joint_layout = QGridLayout(joint_group)
        
        # Create labels for joint display
        self.joint_labels = []
        for i in range(6):
            label = QLabel(f"Joint {i+1}:")
            value = QLabel("0.000°")
            value.setStyleSheet("font-family: monospace; font-weight: bold;")
            joint_layout.addWidget(label, i, 0)
            joint_layout.addWidget(value, i, 1)
            self.joint_labels.append(value)
        
        status_layout.addWidget(joint_group)
        
        # TCP Position display
        tcp_group = QGroupBox("TCP Position")
        tcp_layout = QGridLayout(tcp_group)
        
        # Create labels for TCP display
        self.tcp_labels = {}
        tcp_items = [('X', 'mm'), ('Y', 'mm'), ('Z', 'mm'), ('RX', '°'), ('RY', '°'), ('RZ', '°')]
        
        for i, (name, unit) in enumerate(tcp_items):
            label = QLabel(f"{name}:")
            value_label = QLabel("0.00")
            unit_label = QLabel(unit)
            
            tcp_layout.addWidget(label, i, 0)
            tcp_layout.addWidget(value_label, i, 1)
            tcp_layout.addWidget(unit_label, i, 2)
            
            self.tcp_labels[name.lower()] = value_label
        
        status_layout.addWidget(tcp_group)
        
        # Robot status
        status_group = QGroupBox("Robot Status")
        status_layout_inner = QVBoxLayout(status_group)
        
        self.mode_label = QLabel("Mode: MANUAL")
        self.connection_label = QLabel("Status: Disconnected")
        self.torch_label = QLabel("Torch: OFF")
        
        # Set fixed width to prevent expansion
        self.torch_label.setMinimumWidth(200)
        self.torch_label.setMaximumWidth(200)
        
        self.fault_label = QLabel("Fault: None")
        
        # Pendant connection indicator
        pendant_row = QHBoxLayout()
        self.pendant_status_label = QLabel("Pendant: Not Connected")
        self.pendant_status_label.setStyleSheet("color: #888888; font-weight: bold;")
        pendant_row.addWidget(self.pendant_status_label)
        pendant_row.addStretch()
        
        status_layout_inner.addWidget(self.mode_label)
        status_layout_inner.addWidget(self.connection_label)
        status_layout_inner.addWidget(self.torch_label)
        status_layout_inner.addWidget(self.fault_label)
        status_layout_inner.addLayout(pendant_row)
        
        status_layout.addWidget(status_group)
        
        # Add tabs
        self.status_tabs = status_tabs
        status_tabs.addTab(status_widget, "Status")
        if self.robot_visualizer is not None:
            status_tabs.addTab(self.robot_visualizer, "3D View")
        else:
            # Add a placeholder if 3D visualizer failed to initialize
            placeholder = QLabel("3D Visualization not available")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            status_tabs.addTab(placeholder, "3D View")
        
        layout.addWidget(status_tabs)
        
    def setup_kinematic_worker(self):
        """Initialize and start the kinematic worker thread"""
        self.fk_thread = KinematicThread()
        self.fk_thread.fk_ready.connect(self.on_fk_ready)
        self.fk_thread.ik_ready.connect(self.on_ik_ready)
        self.fk_thread.error.connect(self.on_fk_error)
        
        # Start the thread with buffer support
        self.fk_thread.start()
        
    def request_initial_fk(self):
        """Request initial forward kinematics calculation to populate TCP display"""
        if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
            # Ensure joint input fields are synced with current values
            for i, joint_value in enumerate(self.current_joints):
                if hasattr(self, 'joint_inputs') and i < len(self.joint_inputs):
                    self.joint_inputs[i].setText(f"{joint_value:.1f}")
            
            # Request FK calculation to update TCP display  
            self.fk_thread.request_fk(self.current_joints)
            self.log_event("Initial TCP position calculated", "INFO")
        
    def setup_robot_polling(self):
        """Setup timer for robot state polling"""
        self.robot_timer = QTimer()
        self.robot_timer.timeout.connect(self.update_robot_state)
        self.robot_timer.start(50)  # 20 Hz polling
        
    def setup_jog_button(self, button: QPushButton, axis_id: str, direction: int):
        """Setup a jog button with long press functionality"""
        button_id = f"{axis_id}_{'+' if direction > 0 else '-'}"
        
        # Create timer for this button
        timer = QTimer()
        timer.timeout.connect(lambda: self.execute_jog(axis_id, direction, continuous=True))
        self.jog_timers[button_id] = timer
        self.jog_active[button_id] = False
        self.jog_button_meta[button_id] = (axis_id, direction)
        self.jog_press_time[button_id] = 0.0
        
        # Connect mouse press and release events
        button.pressed.connect(lambda: self.start_jog(button_id, axis_id, direction))
        button.released.connect(lambda: self.stop_jog(button_id))
        
    def start_jog(self, button_id: str, axis_id: str, direction: int):
        """Start continuous jogging when button is pressed"""
        if button_id in self.jog_timers and not self.jog_active.get(button_id, False):
            self.jog_active[button_id] = True
            self.jog_press_time[button_id] = time.time()
            
            # Start movement detection for torch control
            self.on_movement_started()
            
            # Log the jog start
            direction_str = "↑" if direction > 0 else "↓"
            axis_name = axis_id.replace("_", " ").title()
            self.log_event(f"Started continuous jog: {axis_name} {direction_str}", "JOG")
            
            # Hold starts continuous travel after threshold delay.
            QTimer.singleShot(self.jog_initial_delay_ms, lambda: self.continue_jog(button_id))
            
    def continue_jog(self, button_id: str):
        """Continue jogging if button is still pressed"""
        if button_id in self.jog_active and self.jog_active[button_id]:
            axis_id, direction = self.jog_button_meta.get(button_id, (None, None))
            if axis_id is None:
                return
            # First continuous increment immediately, then periodic travel.
            self.execute_jog(axis_id, direction, continuous=True)
            self.jog_timers[button_id].start(self.jog_repeat_interval_ms)
            
    def stop_jog(self, button_id: str):
        """Stop continuous jogging when button is released"""
        if button_id in self.jog_timers:
            was_active = self.jog_active.get(button_id, False)
            pressed_for = time.time() - float(self.jog_press_time.get(button_id, 0.0))
            self.jog_active[button_id] = False
            self.jog_timers[button_id].stop()

            # Quick tap -> single fine step (mm/step or deg/step).
            if was_active and pressed_for < self.jog_hold_threshold_s:
                axis_id, direction = self.jog_button_meta.get(button_id, (None, None))
                if axis_id is not None:
                    self.execute_single_jog(axis_id, direction)
            
            # Log the jog stop
            self.log_event("Stopped continuous jog", "JOG")
            
            # Check if any jog buttons are still active
            any_jog_active = any(self.jog_active.values())
            if not any_jog_active:
                self._cart_jog_elbow_branch = None
                # No jog buttons active, trigger movement stop detection
                self.movement_timeout_timer.stop()
                self.movement_timeout_timer.start(200)  # Shorter timeout for button release
    
    def _continuous_jog_delta(self, axis_id: str, direction: int) -> float:
        """Compute per-tick travel delta from speed and jog timer interval."""
        dt = self.jog_repeat_interval_ms / 1000.0
        if axis_id.startswith("joint_"):
            speed = min(self.movement_speed, 20.0)  # cap joint jog at 20 deg/s
            return direction * max(0.05, speed * dt)  # deg per tick
        base = max(0.01, self.movement_speed * dt)
        return direction * max(0.05, base)      # mm or deg per tick

    def _snapshot_physical_jog_start(self):
        """Use hardware feedback when present so the preplan starts from the real robot."""
        feedback_joints = getattr(self, '_feedback_current_joints', None)
        use_feedback = bool(getattr(self, 'button_states', {}).get('connect'))
        if use_feedback and isinstance(feedback_joints, (list, tuple)) and len(feedback_joints) == 6:
            start_joints = [float(v) for v in feedback_joints]
        else:
            start_joints = [float(v) for v in self.current_joints]

        try:
            start_tcp = tuple(float(v) for v in solve_fk_gui(start_joints))
        except Exception:
            start_tcp = tuple(float(v) for v in self.tcp_pose)
        return start_joints, start_tcp

    def _queue_physical_jog_preplan(
        self,
        axis: str,
        start_joints,
        target_joints,
        start_tcp=None,
        target_tcp=None,
        motion_source: str = "manual",
        transition_mode: str = None,
    ):
        """Debounce manual motion requests into a single offline jog preplan."""
        if not getattr(self, 'physical_jog_preplan_enabled', False):
            return

        if start_joints is None or target_joints is None:
            return

        start_joints = [float(v) for v in start_joints]
        target_joints = [float(v) for v in target_joints]
        if len(start_joints) != 6 or len(target_joints) != 6:
            return

        max_delta = max(abs(target_joints[i] - start_joints[i]) for i in range(6))
        if max_delta <= 1e-6:
            return

        if start_tcp is None:
            try:
                start_tcp = tuple(float(v) for v in solve_fk_gui(start_joints))
            except Exception:
                start_tcp = tuple(float(v) for v in self.tcp_pose)
        if target_tcp is None:
            try:
                target_tcp = tuple(float(v) for v in solve_fk_gui(target_joints))
            except Exception:
                target_tcp = tuple(float(v) for v in self.tcp_pose)

        if transition_mode is None:
            if str(axis).startswith("cart_") and self.transition_mode == "LINEAR":
                transition_mode = "LINEAR"
            else:
                transition_mode = "P2P"

        self._jog_preplan_request_seq += 1
        self._pending_jog_preplan_request = {
            "request_id": self._jog_preplan_request_seq,
            "motion_source": motion_source,
            "axis": str(axis),
            "speed": float(self.movement_speed),
            "plan_rate_hz": float(self.physical_jog_plan_rate_hz),
            "samples_per_direction": int(self.jog_knowledge_samples_per_direction),
            "position_span_mm": float(self.jog_knowledge_position_span_mm),
            "orientation_span_deg": float(self.jog_knowledge_orientation_span_deg),
            "start_joints": start_joints,
            "target_joints": target_joints,
            "start_tcp": tuple(float(v) for v in start_tcp),
            "target_tcp": tuple(float(v) for v in target_tcp),
            "seed_joints": target_joints,
            "seed_tcp": tuple(float(v) for v in target_tcp),
            "output_dir": os.path.join(self.positions_base_dir, "_temp_jog_preplans"),
        }
        if hasattr(self, 'jog_preplan_timer'):
            self.jog_preplan_timer.start(self.physical_jog_plan_debounce_ms)

    def _dispatch_jog_preplan(self):
        """Start the latest pending jog preplan in a separate thread."""
        request = self._pending_jog_preplan_request
        if not request:
            return

        thread = getattr(self, 'jog_preplan_thread', None)
        if thread is not None and thread.isRunning():
            thread.requestInterruption()
            self.jog_preplan_timer.start(50)
            return

        self._pending_jog_preplan_request = None
        self.jog_preplan_thread = JogPreplannerThread(request, self)
        self.jog_preplan_thread.plan_ready.connect(self._on_jog_preplan_ready)
        self.jog_preplan_thread.log_message.connect(self._on_jog_preplan_log)
        self.jog_preplan_thread.error.connect(self._on_jog_preplan_error)
        self.jog_preplan_thread.finished.connect(self._on_jog_preplan_finished)
        self.jog_preplan_thread.start()

    @pyqtSlot(dict)
    def _on_jog_preplan_ready(self, plan: dict):
        """Keep only the newest generated physical jog preplan."""
        request_id = int(plan.get("request_id", 0))
        if request_id < int(getattr(self, '_jog_preplan_request_seq', 0)):
            return

        self._latest_jog_preplan = plan
        self._latest_jog_preplan_file = plan.get("file_path")
        file_name = os.path.basename(str(self._latest_jog_preplan_file or ""))
        summary = plan.get("axis_summary", {})
        button_summary = plan.get("button_summary", {})
        requested_button = str(plan.get("requested_button", "") or "")
        covered_axes = sum(
            1
            for axis_data in summary.values()
            if int(axis_data.get("neg_samples", 0)) > 0 or int(axis_data.get("pos_samples", 0)) > 0
        )
        covered_buttons = sum(
            1 for button_data in button_summary.values()
            if int(button_data.get("samples", 0)) > 0
        )
        self.log_event(
            f"Jog knowledge ready: {plan.get('frame_count', 0)} samples, "
            f"{covered_axes}/6 axes, {covered_buttons}/12 buttons"
            f"{f' [{requested_button}]' if requested_button else ''} -> {file_name}",
            "INFO",
        )

    @pyqtSlot(str, str)
    def _on_jog_preplan_log(self, message: str, level: str):
        self.log_event(message, level)

    @pyqtSlot(str)
    def _on_jog_preplan_error(self, message: str):
        self.log_event(message, "ERROR")

    @pyqtSlot()
    def _on_jog_preplan_finished(self):
        thread = getattr(self, 'jog_preplan_thread', None)
        if thread is not None:
            thread.deleteLater()
        self.jog_preplan_thread = None
        if self._pending_jog_preplan_request is not None:
            self.jog_preplan_timer.start(10)

    def execute_jog(self, axis_id: str, direction: int, continuous: bool = False):
        """Execute a jog movement for the specified axis"""
        if continuous:
            delta = self._continuous_jog_delta(axis_id, direction)
        elif axis_id.startswith("joint_"):
            delta = direction * self.joint_step_size
        else:
            delta = direction * self.cartesian_step_size

        if axis_id.startswith("joint_"):
            joint_idx = int(axis_id.split("_")[1])
            self.jog_joint(joint_idx, delta, log_move=not continuous)
        elif axis_id.startswith("cart_"):
            axis = axis_id.split("_")[1]
            self.jog_cartesian(axis, delta, log_move=not continuous)
            
    def execute_single_jog(self, axis_id: str, direction: int):
        """Execute a single jog movement (for button clicks) with torch control"""
        # Start movement detection for torch control
        self.on_movement_started()
        
        # Execute the actual jog
        self.execute_jog(axis_id, direction)
        
        # Set a short timeout to turn off torch after single movement
        self.movement_timeout_timer.stop()
        self.movement_timeout_timer.start(300)  # 300ms timeout for single clicks
    
    def jog_joint(self, joint_index: int, delta: float, log_move: bool = True):
        """Jog a specific joint by delta degrees with soft limits."""
        if 0 <= joint_index < 6:
            start_joints, start_tcp = self._snapshot_physical_jog_start()
            self._pending_cartesian_fk_lock = None
            current_value = self.current_joints[joint_index]
            new_value = current_value + delta
            self.current_joints[joint_index] = new_value
            
            # Log joint movement
            if log_move:
                direction = "↑" if delta > 0 else "↓"
                self.log_event(f"Joint {joint_index+1} {direction} → {new_value:.1f}°", "JOG")
            
            # Update the input field to show current position
            self.joint_inputs[joint_index].setText(f"{new_value:.1f}")
            
            # Update joint labels in status panel
            if hasattr(self, 'joint_labels') and joint_index < len(self.joint_labels):
                self.joint_labels[joint_index].setText(f"{new_value:.3f}°")
            
            # Update 3D visualization with new joint angles
            if hasattr(self, 'robot_visualizer') and self.robot_visualizer is not None:
                self.robot_visualizer.update_joints(self.current_joints)
            
            # Always request FK so Cartesian display stays in sync
            # (the worker debounces internally, keeping only the latest request)
            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self.fk_thread.request_fk(self.current_joints)

            self._queue_physical_jog_preplan(
                axis=f"joint_{joint_index + 1}",
                start_joints=start_joints,
                target_joints=self.current_joints.copy(),
                start_tcp=start_tcp,
                motion_source="jog_joint",
                transition_mode="P2P",
            )

    def jog_cartesian(self, axis: str, delta: float, log_move: bool = True):
        """Jog a single GUI Cartesian axis.

        Position axes (x/y/z) are moved in the base frame.
        Orientation axes (rx/ry/rz) are rotated about the current TCP frame
        via post-multiply: R_new = R_actual * R_delta.

        The IK target is always built from FK of current joints so that
        stored display Euler values never feed back into the solver —
        this prevents orientation snaps when switching between axes.
        """
        start_joints, start_tcp = self._snapshot_physical_jog_start()
        tcp_backup = tuple(self.tcp_pose)
        axis_map = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}
        idx = axis_map.get(axis)
        if idx is None:
            return

        # ── Jog-knowledge reach guard ────────────────────────────────────────
        # Consult the preplan knowledge map to catch moves that approach or
        # exceed the IK-reachable workspace in this direction before we run
        # the solver (fail-fast, avoids wasted FK+IK calls on blocked axes).
        risk_status, risk_msg = self._check_jog_direction_reach(axis, float(delta))
        if risk_status == 'block':
            self.log_event(risk_msg, "WARNING")
            return
        if risk_status == 'warn':
            self.log_event(risk_msg, "WARNING")
        # ─────────────────────────────────────────────────────────────────────

        # Always start from FK truth so cosmetic display values never
        # contaminate the IK target.
        fk_pose = solve_fk_gui(self.current_joints)
        ik_tcp = list(fk_pose)       # IK target — built from FK truth
        display_tcp = list(self.tcp_pose)  # display — only jogged axis changes

        if idx < 3:
            # Position jog: base-frame offset.
            ik_tcp[idx] += float(delta)
            display_tcp[idx] += float(delta)
        else:
            # Orientation jog: rotate about the TCP (tool) frame axis.
            R_cur = R_scipy.from_euler("xyz",
                                       [fk_pose[3], fk_pose[4], fk_pose[5]],
                                       degrees=True)
            tool_axis_idx = idx - 3  # 0=x, 1=y, 2=z
            small_vec = [0.0, 0.0, 0.0]
            small_vec[tool_axis_idx] = float(delta)
            R_delta = R_scipy.from_euler("xyz", small_vec, degrees=True)
            R_new = R_cur * R_delta
            new_euler = R_new.as_euler("xyz", degrees=True)
            ik_tcp[3] = self._wrap_to_180(float(new_euler[0]))
            ik_tcp[4] = self._wrap_to_180(float(new_euler[1]))
            ik_tcp[5] = self._wrap_to_180(float(new_euler[2]))
            # Display: only the jogged axis increments
            display_tcp[idx] = self._wrap_to_180(display_tcp[idx] + float(delta))

        ik_target = tuple(ik_tcp)

        if idx >= 3:
            # Orientation jog: seed IK toward the active wrist joint, then
            # unwrap the solution near current joints for continuity.
            axis_to_wrist = {'rx': 3, 'ry': 4, 'rz': 5}
            wrist_idx = axis_to_wrist.get(axis, 5)
            seed = [float(v) for v in self.current_joints]
            seed[wrist_idx] += float(delta)
            sol = solve_ik_gui(ik_target, seed=seed)
            if sol is None:
                sol = solve_ik_gui(ik_target, seed=self.current_joints)
            if sol is not None:
                sol = self._unwrap_joints_near(sol, self.current_joints)
        else:
            sol = solve_ik_gui(ik_target, seed=self.current_joints)
        if sol is None:
            self.tcp_pose = tcp_backup
            self.log_event(f"Cartesian {axis.upper()} unreachable (IK fail)", "WARNING")
            return
        self.current_joints = [float(v) for v in sol]
        self._sync_unwrapped_wrist()

        if hasattr(self, 'joint_inputs'):
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_inputs):
                    self.joint_inputs[i].setText(f"{jv:.2f}")
        if hasattr(self, 'joint_labels'):
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_labels):
                    self.joint_labels[i].setText(f"{jv:.3f}°")
        if hasattr(self, 'robot_visualizer') and self.robot_visualizer is not None:
            self.robot_visualizer.update_joints(self.current_joints)

        commanded_pose = tuple(float(v) for v in display_tcp)
        self._pending_cartesian_fk_lock = {
            'pose': commanded_pose,
            'joints': tuple(float(v) for v in self.current_joints),
        }
        self.tcp_pose = self._stabilize_tcp_orientation(commanded_pose)
        self.last_fk_joints = self.current_joints.copy()

        self._update_tcp_labels_from_pose(self.tcp_pose)
        self.update_cartesian_display()
        self._queue_physical_jog_preplan(
            axis=f"cart_{axis}",
            start_joints=start_joints,
            target_joints=self.current_joints.copy(),
            start_tcp=start_tcp,
            target_tcp=commanded_pose,
            motion_source="jog_cartesian",
        )
        return
    
    def update_cartesian_display(self):
        """Update cartesian input values to show current position (throttled)"""
        x, y, z, rx, ry, rz = self._pose_for_display(self.tcp_pose)
        self.cartesian_inputs['x'].setText(f"{x:.2f}")
        self.cartesian_inputs['y'].setText(f"{y:.2f}")
        self.cartesian_inputs['z'].setText(f"{z:.2f}")
        self.cartesian_inputs['rx'].setText(f"{rx:.2f}")
        self.cartesian_inputs['ry'].setText(f"{ry:.2f}")
        self.cartesian_inputs['rz'].setText(f"{rz:.2f}")

    def schedule_cartesian_display_update(self):
        """Coalesce frequent TCP field updates for smoother UI."""
        if not hasattr(self, 'cartesian_update_timer'):
            return
        if not self.cartesian_update_timer.isActive():
            self.cartesian_update_timer.start(self.cartesian_update_interval_ms)
    
    def load_jog_data(self):
        """Load jog data from JSON file including record positions and home"""
        try:
            if os.path.exists(self.jog_data_file):
                with open(self.jog_data_file, 'r') as f:
                    data = json.load(f)

                loaded_joints = None
                if 'joints' in data and len(data['joints']) == 6:
                    loaded_joints = [float(v) for v in data['joints']]

                # Load TCP pose
                if 'tcp_pose' in data and len(data['tcp_pose']) == 6:
                    self.tcp_pose = self._canonicalize_tcp_pose(tuple(float(v) for v in data['tcp_pose']))
                    # Update cartesian input fields
                    if hasattr(self, 'cartesian_inputs'):
                        axes = ['x', 'y', 'z', 'rx', 'ry', 'rz']
                        for i, axis in enumerate(axes):
                            if axis in self.cartesian_inputs:
                                self.cartesian_inputs[axis].setText(f"{self.tcp_pose[i]:.2f}")

                # Always reconcile saved joints against the active IK backend.
                solved_current = self._solve_gui_pose_joints(self.tcp_pose)
                if solved_current is not None:
                    self.current_joints = solved_current
                elif loaded_joints is not None:
                    self.current_joints = loaded_joints

                for i, joint_value in enumerate(self.current_joints):
                    if hasattr(self, 'joint_inputs') and i < len(self.joint_inputs):
                        self.joint_inputs[i].setText(f"{joint_value:.1f}")
                    if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                        self.joint_labels[i].setText(f"{joint_value:.3f}°")

                # Load record positions (JSON str keys -> int keys)
                if 'record_positions' in data and data['record_positions']:
                    for k, v in data['record_positions'].items():
                        tcp_pose = tuple(float(x) for x in v['tcp_pose'])
                        solved_joints = self._solve_gui_pose_joints(tcp_pose)
                        self.record_positions[int(k)] = {
                            'joints': solved_joints if solved_joints is not None else v['joints'],
                            'unwrapped_wrist': v.get('unwrapped_wrist', [0.0, 0.0, 0.0]),
                            'tcp_pose': tcp_pose,
                            'timestamp': v.get('timestamp', 0)
                        }
                
                # Load home position
                if 'home_position' in data and data['home_position'] is not None:
                    hp = data['home_position']
                    tcp_pose = tuple(float(x) for x in hp['tcp_pose'])
                    solved_joints = self._solve_gui_pose_joints(tcp_pose)
                    self.home_position = {
                        'joints': solved_joints if solved_joints is not None else hp['joints'],
                        'unwrapped_wrist': hp.get('unwrapped_wrist', [0.0, 0.0, 0.0]),
                        'tcp_pose': tcp_pose,
                        'timestamp': hp.get('timestamp', 0)
                    }
                
                print(f"Loaded jog data from {self.jog_data_file}")
        except Exception as e:
            print(f"Error loading jog data: {e}")
    
    def save_jog_data(self):
        """Save current jog data to JSON file including record positions and home"""
        try:
            # Convert record positions (int keys -> str keys for JSON)
            record_data = {}
            for k, v in self.record_positions.items():
                record_data[str(k)] = {
                    'joints': v['joints'],
                    'tcp_pose': list(v['tcp_pose']) if isinstance(v['tcp_pose'], tuple) else v['tcp_pose'],
                    'timestamp': v.get('timestamp', 0)
                }
            
            # Convert home position
            home_data = None
            if self.home_position is not None:
                home_data = {
                    'joints': self.home_position['joints'],
                    'tcp_pose': list(self.home_position['tcp_pose']) if isinstance(self.home_position['tcp_pose'], tuple) else self.home_position['tcp_pose'],
                    'timestamp': self.home_position.get('timestamp', 0)
                }
            
            data = {
                'joints': self.current_joints,
                'tcp_pose': list(self._canonicalize_tcp_pose(self.tcp_pose)),
                'record_positions': record_data,
                'home_position': home_data,
                'timestamp': time.time()
            }
            with open(self.jog_data_file, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"Saved jog data to {self.jog_data_file}")
        except Exception as e:
            print(f"Error saving jog data: {e}")
    
    def sync_jog_values(self):
        """Sync joint and cartesian values using forward kinematics"""
        try:
            # Use current joint values to calculate TCP pose via FK
            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self._pending_cartesian_fk_lock = None
                # Force update cartesian display from current joints
                self.fk_thread.request_fk(self.current_joints)
                # Also update the cartesian inputs immediately from current tcp_pose
                self.update_cartesian_display()
                self.status_bar.showMessage("Syncing joint and cartesian values...", 2000)
            else:
                self.status_bar.showMessage("FK thread not available for sync", 3000)
        except Exception as e:
            print(f"Error syncing values: {e}")
            self.status_bar.showMessage(f"Sync error: {e}", 3000)
    
    def on_joint_step_changed(self, value):
        """Handle joint step size slider change"""
        self.joint_step_size = value / 10.0  # Convert to 0.1 - 10.0 range
        self.joint_step_input.setText(f"{self.joint_step_size:.1f}")
    
    def on_joint_step_input_changed(self, text):
        """Handle joint step size input change"""
        try:
            value = float(text)
            if 0.1 <= value <= 10.0:
                self.joint_step_size = value
                self.joint_step_slider.setValue(int(value * 10))
        except ValueError:
            pass
    
    def on_cartesian_step_changed(self, value):
        """Handle cartesian step size slider change"""
        self.cartesian_step_size = value / 10.0  # Convert to 0.1 - 10.0 range
        self.cartesian_step_input.setText(f"{self.cartesian_step_size:.1f}")
    
    def on_cartesian_step_input_changed(self, text):
        """Handle cartesian step size input change"""
        try:
            value = float(text)
            if 0.1 <= value <= 10.0:
                self.cartesian_step_size = value
                self.cartesian_step_slider.setValue(int(value * 10))
        except ValueError:
            pass
    
    def on_speed_changed(self, value):
        """Handle movement speed slider change"""
        self.movement_speed = float(value)
        self.speed_input.setText(f"{value:.1f}")
    
    def on_speed_input_changed(self, text):
        """Handle movement speed input field change"""
        try:
            value = float(text)
            if 1.0 <= value <= 200.0:
                self.movement_speed = value
                self.speed_slider.setValue(int(value))
        except ValueError:
            pass
    
    def toggle_weld_state(self):
        """Toggle welding on/off state with mode-specific behavior"""
        self.welding_enabled = self.weld_toggle_btn.isChecked()
        
        # Check current mode to determine behavior
        jog_mode_active = self.button_states.get('jog', False)
        teach_mode_active = self.button_states.get('teach', False)
        
        if self.welding_enabled:
            self.weld_toggle_btn.setText("Weld ON")
            
            if jog_mode_active:
                # In JOG mode: Torch will activate only during movement
                self.log_event("Weld enabled in JOG mode - torch will activate during movement", "WELD")
                # Check if currently moving to activate torch immediately
                self.update_torch_for_movement()
            elif teach_mode_active:
                # In TEACH mode: Only set weld parameter for program entries
                self.torch_active = False
                self.log_event("Weld parameter set ON for program positions (torch not active)", "PROGRAM")
            else:
                # No active mode: Just parameter setting
                self.torch_active = False
                self.log_event("Weld parameter enabled (no active mode)", "WELD")
        else:
            self.weld_toggle_btn.setText("Weld OFF")
            
            if jog_mode_active and self.torch_active:
                # In JOG mode: Turn off the torch
                self.torch_active = False
                self.log_event("Weld disabled - torch deactivated", "WELD")
            elif teach_mode_active:
                # In TEACH mode: Just parameter setting
                self.log_event("Weld parameter set OFF for program positions", "PROGRAM")
            else:
                # No active mode or torch wasn't active
                self.torch_active = False
                self.log_event("Weld parameter disabled", "WELD")
                
        # Update torch status display
        self.update_torch_status_display()
    
    def set_motion_type(self, motion_type):
        """Set the motion type for program positions"""
        self.current_motion_type = motion_type
        
        # Update button states (mutual exclusivity)
        self.linear_btn.setChecked(motion_type == "LINEAR")
        self.curve_btn.setChecked(motion_type == "CURVE")
        self.p2p_btn.setChecked(motion_type == "P2P")
        
        # Log motion type change
        self.log_event(f"Motion type changed to {motion_type}", "JOG")
        
        # Update status
        self.status_bar.showMessage(f"Motion type set to: {motion_type}")
    
    def _on_transition_mode_changed(self, text):
        """Handle transition mode combo change."""
        self.transition_mode = text.upper()
        self.log_event(f"Transition mode → {self.transition_mode}", "JOG")
        self.status_bar.showMessage(f"Transition mode: {self.transition_mode}")

    def go_to_joint_target(self, joint_index):
        """Move joint to target position"""
        try:
            start_joints, start_tcp = self._snapshot_physical_jog_start()
            self._pending_cartesian_fk_lock = None
            target = float(self.joint_inputs[joint_index].text())
            target = max(-180.0, min(180.0, target))  # Clamp to valid range
            self.current_joints[joint_index] = target
            
            # Update the input field to show the clamped value
            self.joint_inputs[joint_index].setText(f"{target:.1f}")
            
            # Update joint label in status panel
            if hasattr(self, 'joint_labels') and joint_index < len(self.joint_labels):
                self.joint_labels[joint_index].setText(f"{target:.3f}°")
            
            # Update 3D visualization
            if hasattr(self, 'robot_visualizer') and self.robot_visualizer is not None:
                self.robot_visualizer.update_joints(self.current_joints)
            
            # Request FK calculation to update TCP display
            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self.fk_thread.request_fk(self.current_joints)

            self._queue_physical_jog_preplan(
                axis=f"joint_{joint_index + 1}",
                start_joints=start_joints,
                target_joints=self.current_joints.copy(),
                start_tcp=start_tcp,
                motion_source="joint_target",
                transition_mode="P2P",
            )
        except ValueError:
            # Reset to current value if invalid input
            self.joint_inputs[joint_index].setText(f"{self.current_joints[joint_index]:.1f}")
    
    def go_to_cartesian_target(self, axis):
        """Move to cartesian target position using inverse kinematics"""
        start_joints, start_tcp = self._snapshot_physical_jog_start()
        try:
            target = float(self.cartesian_inputs[axis].text())
        except ValueError:
            if axis in self.cartesian_inputs:
                axis_map = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}
                if axis in axis_map:
                    idx = axis_map[axis]
                    self.cartesian_inputs[axis].setText(f"{self._pose_for_display(self.tcp_pose)[idx]:.2f}")
            return

        axis_map = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}
        if axis not in axis_map:
            return

        tcp_backup = tuple(self.tcp_pose)
        idx = axis_map[axis]
        if axis in ('rx', 'ry', 'rz'):
            target = self._wrap_to_180(target)

        target_tcp = list(self.tcp_pose)
        target_tcp[idx] = float(target)

        sol = solve_ik_gui(tuple(target_tcp), seed=self.current_joints)
        if sol is None:
            self.tcp_pose = tcp_backup
            self.update_cartesian_display()
            self.log_event(f"Cartesian target {axis.upper()} unreachable", "WARNING")
            return

        self.current_joints = [float(v) for v in sol]
        self._sync_unwrapped_wrist()

        if hasattr(self, 'joint_inputs'):
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_inputs):
                    self.joint_inputs[i].setText(f"{jv:.2f}")
        if hasattr(self, 'joint_labels'):
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_labels):
                    self.joint_labels[i].setText(f"{jv:.3f}°")
        if hasattr(self, 'robot_visualizer') and self.robot_visualizer is not None:
            self.robot_visualizer.update_joints(self.current_joints)

        commanded_pose = tuple(float(v) for v in target_tcp)
        self._pending_cartesian_fk_lock = {
            'pose': commanded_pose,
            'joints': tuple(float(v) for v in self.current_joints),
        }
        self.tcp_pose = self._stabilize_tcp_orientation(commanded_pose)
        self.last_fk_joints = self.current_joints.copy()

        self.update_cartesian_display()
        self._update_tcp_labels_from_pose(self.tcp_pose)

        self._last_axis_jog_state = {'axis': axis, 'joints': self.current_joints.copy()}

        unit = "mm" if axis in ('x', 'y', 'z') else "°"
        self.log_event(f"Cartesian target {axis.upper()} -> {target:.2f}{unit}", "JOG")
        self._queue_physical_jog_preplan(
            axis=f"cart_{axis}",
            start_joints=start_joints,
            target_joints=self.current_joints.copy(),
            start_tcp=start_tcp,
            target_tcp=commanded_pose,
            motion_source="cartesian_target",
        )
        return
            
    @staticmethod
    def _unwrap_angle_near(value: float, reference: float) -> float:
        """Return an angle equivalent to value but closest to reference."""
        v = float(value)
        r = float(reference)
        while v - r > 180.0:
            v -= 360.0
        while v - r < -180.0:
            v += 360.0
        return v
    
    def _sync_unwrapped_wrist(self):
        """Update unwrapped_wrist to match current_joints[4:6], preserving continuity."""
        for i in range(3):
            wrapped = float(self.current_joints[3 + i])
            unwrapped = self._unwrap_angle_near(wrapped, self.unwrapped_wrist[i])
            self.unwrapped_wrist[i] = unwrapped

    def _unwrap_joints_near(self, candidate: list, reference) -> list:
        """Unwind each joint around a reference frame."""
        ref = np.asarray(reference, dtype=float).reshape(6)
        cand = np.asarray(candidate, dtype=float).reshape(6)
        return [
            float(ref[i] + (((cand[i] - ref[i] + 180.0) % 360.0) - 180.0))
            for i in range(6)
        ]

    @staticmethod
    def _elbow_jump_exceeds(prev: list, cand: list, max_deg: float = 12.0) -> bool:
        """Return True if J2 or J3 changes by more than max_deg."""
        p = np.asarray(prev, dtype=float).reshape(6)
        c = np.asarray(cand, dtype=float).reshape(6)
        return bool(
            abs(float(c[1] - p[1])) > float(max_deg)
            or abs(float(c[2] - p[2])) > float(max_deg)
        )

    @staticmethod
    def _wrap_to_180(value: float) -> float:
        """Map angle to [-180, 180] for Cartesian display/input."""
        return ((float(value) + 180.0) % 360.0) - 180.0

    @staticmethod
    def _quat_orientation_distance_deg(euler_a, euler_b) -> float:
        """Shortest rotation angle between two XYZ Euler orientations in degrees."""
        ra = R_scipy.from_euler("xyz", [float(v) for v in euler_a], degrees=True)
        rb = R_scipy.from_euler("xyz", [float(v) for v in euler_b], degrees=True)
        delta = ra.inv() * rb
        return float(np.degrees(np.linalg.norm(delta.as_rotvec())))

    def _sample_orientation_slerp(self, start_euler, target_euler, t_values):
        """Sample quaternion-slerped XYZ Euler orientations with unwrap continuity."""
        start_rot = R_scipy.from_euler("xyz", [float(v) for v in start_euler], degrees=True)
        target_rot = R_scipy.from_euler("xyz", [float(v) for v in target_euler], degrees=True)
        self.linear_slerp = Slerp(
            [0.0, 1.0],
            R_scipy.from_quat(np.vstack([start_rot.as_quat(), target_rot.as_quat()])),
        )

        samples = []
        prev = None
        for t in t_values:
            euler = self.linear_slerp([float(t)]).as_euler("xyz", degrees=True)[0]
            if prev is not None:
                euler = np.array([
                    self._unwrap_angle_near(euler[idx], prev[idx])
                    for idx in range(3)
                ], dtype=float)
            sample = (float(euler[0]), float(euler[1]), float(euler[2]))
            samples.append(sample)
            prev = sample
        return samples

    @staticmethod
    def _solve_3r_position_yaskawa(x_mm: float, y_mm: float, z_mm: float) -> list:
        """Analytic 3R position IK candidates (J1-J3) in Yaskawa convention.

        Uses the provided geometry and corrected radial projection:
        exd = x/cos(theta1) - a1, equivalent to hypot(x, y) - a1.
        Returns up to two elbow branches as (j1, j2, j3) in degrees.
        """
        d1 = 450.0
        a1 = 155.0
        a2 = 614.0
        a3 = float(np.hypot(200.0, 640.0))
        alpha = float(np.arctan2(200.0, 640.0))

        theta1 = float(np.arctan2(y_mm, x_mm))

        # Corrected precedence version: x/cos(theta1) - a1.
        # Use radial equivalent for robustness near x ~= 0.
        exd = float(np.hypot(x_mm, y_mm) - a1)
        ezd = float(z_mm - d1)

        v2 = (exd * exd + ezd * ezd) - (a2 * a2 + a3 * a3)
        v3 = 2.0 * a2 * a3
        if abs(v3) < 1e-9:
            return []

        v4 = float(np.clip(v2 / v3, -1.0, 1.0))

        solutions = []
        for theta3_raw in (-np.arccos(v4), np.arccos(v4)):
            theta3 = float(theta3_raw)
            theta2 = float(
                np.arctan2(ezd, exd)
                - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
            )

            j1 = float(np.degrees(theta1))
            j2 = float(np.degrees(theta2))
            j3 = float(np.degrees(theta3 + (np.pi / 2.0 - alpha)))
            solutions.append((j1, j2, j3))

        return solutions

    def _pose_for_display(self, pose):
        """Return pose with orientation wrapped to [-180, 180]."""
        return self._canonicalize_tcp_pose(pose)

    def _canonicalize_tcp_pose(self, pose):
        """Keep stored GUI poses in canonical wrapped Euler form."""
        x, y, z, rx, ry, rz = pose
        return (
            float(x),
            float(y),
            float(z),
            self._wrap_to_180(rx),
            self._wrap_to_180(ry),
            self._wrap_to_180(rz),
        )

    def _update_tcp_labels_from_pose(self, pose):
        """Refresh the status-panel TCP labels from a GUI-convention pose."""
        display_pose = self._pose_for_display(pose)
        for idx, label in enumerate(('x', 'y', 'z', 'rx', 'ry', 'rz')):
            self._set_text_if_changed(self.tcp_labels[label], f"{display_pose[idx]:.2f}")

    @staticmethod
    def _set_text_if_changed(widget, text: str):
        """Avoid redundant setText calls in high-frequency UI update paths."""
        if widget is None:
            return
        try:
            if widget.text() != text:
                widget.setText(text)
        except Exception:
            widget.setText(text)

    def _display_pose_from_fk(self, pose):
        """Return the GUI pose to expose after FK, honoring a single-axis Cartesian lock."""
        normalized = self._normalize_fk_pose(pose)
        lock = self._pending_cartesian_fk_lock
        if not lock:
            return self._stabilize_tcp_orientation(normalized)

        lock_pose = lock.get('pose')
        lock_joints = lock.get('joints')
        if lock_pose is None or lock_joints is None or len(lock_joints) != len(self.current_joints):
            self._pending_cartesian_fk_lock = None
            return self._stabilize_tcp_orientation(normalized)

        joint_error = max(
            abs(float(self.current_joints[i]) - float(lock_joints[i]))
            for i in range(len(self.current_joints))
        )
        if joint_error > 1e-4:
            self._pending_cartesian_fk_lock = None
            return self._stabilize_tcp_orientation(normalized)

        return self._stabilize_tcp_orientation(tuple(float(v) for v in lock_pose))

    def _normalize_fk_pose(self, pose, reference=None):
        """Return a GUI-order pose without applying extra remapping logic."""
        del reference
        return tuple(float(v) for v in pose)

    def select_best_solution(self, solutions, prev_angles, max_joint_step_deg=35.0):
        """Pick the most continuous IK candidate and track rejects.

        Selection rule:
        min(sum((q_current - q_prev)^2))
        using wrap-aware unwrapping around q_prev.
        """
        if not solutions:
            return None, {'total': 0, 'accepted': 0, 'rejected': 0, 'relaxed': False, 'max_step': 0.0}

        prev = np.array([float(v) for v in prev_angles], dtype=float)
        rows = []
        for s in solutions:
            q = np.array([float(v) for v in s], dtype=float)
            qn = np.array([
                prev[i] + (((q[i] - prev[i] + 180.0) % 360.0) - 180.0)
                for i in range(6)
            ], dtype=float)
            delta = np.abs(qn - prev)
            max_step = float(np.max(delta))
            cost = float(np.sum((qn - prev) ** 2))
            rows.append((qn, cost, max_step, max_step <= float(max_joint_step_deg)))

        accepted = [r for r in rows if r[3]]
        pool = accepted if accepted else rows
        best = min(pool, key=lambda r: r[1])
        info = {
            'total': len(rows),
            'accepted': len(accepted),
            'rejected': len(rows) - len(accepted),
            'relaxed': len(accepted) == 0,
            'max_step': best[2],
        }
        return best[0].tolist(), info

    def _stabilize_tcp_orientation(self, pose):
        """Return a canonical pose without accumulating unwrapped Euler drift."""
        stable = self._canonicalize_tcp_pose(pose)
        self._last_display_tcp_pose = stable
        return stable

    def _motion_context_active(self) -> bool:
        """Return True while any explicit motion/jog/playback is active."""
        jog_running = any(self.jog_active.values()) if hasattr(self, 'jog_active') else False
        traj_running = bool(getattr(getattr(self, 'robot_visualizer', None), 'traj_playing', False))
        return bool(
            jog_running
            or getattr(self, 'movement_active', False)
            or getattr(self, 'p2p_active', False)
            or getattr(self, 'linear_active', False)
            or traj_running
        )

    def _vk_solve_pose(self, pose):
        """Solve IK through Visual Kinematics for a GUI-convention pose."""
        sol = solve_ik_gui(pose, seed=self.current_joints)
        if sol is None:
            return None
        return np.array([float(v) for v in sol], dtype=float)

    def _solve_gui_pose_joints(self, pose):
        """Solve joint angles for a GUI-convention pose using the active backend."""
        try:
            gui_pose = tuple(float(v) for v in pose)
        except Exception:
            return None
        sol = solve_ik_gui(gui_pose, seed=self.current_joints)
        if sol is None:
            return None
        return [float(v) for v in sol]

    def _check_jog_direction_reach(self, axis: str, delta: float):
        """Check the latest jog knowledge map for reach risk on axis+direction.

        Returns a tuple (status, message):
          'ok'    — sufficient reach, proceed normally
          'warn'  — within 3 steps of workspace limit, allow but log
          'block' — no reach left, caller should abort this jog step

        The check is skipped (returns 'ok') when no preplan exists yet.
        """
        plan = getattr(self, '_latest_jog_preplan', None)
        if not plan:
            return ('ok', '')

        btn_id = f"cart_{axis}{'+'if delta > 0 else '-'}"
        button_summary = plan.get('button_summary', {})
        meta = button_summary.get(btn_id)
        if meta is None:
            return ('ok', '')

        samples = int(meta.get('samples', 0))
        reach = abs(float(meta.get('reach', 0.0)))
        step = abs(float(delta))

        if samples == 0 or (step > 0 and reach < step * 0.5):
            unit = 'mm' if axis in ('x', 'y', 'z') else 'deg'
            return (
                'block',
                f"Cartesian {axis.upper()}{'+'if delta>0 else '-'} blocked: "
                f"workspace limit reached (reach={reach:.1f}{unit}, step={step:.1f}{unit})",
            )
        if step > 0 and reach < step * 3.0:
            unit = 'mm' if axis in ('x', 'y', 'z') else 'deg'
            return (
                'warn',
                f"Cartesian {axis.upper()}{'+'if delta>0 else '-'} approaching limit "
                f"(~{reach/step:.0f} steps remaining, reach={reach:.1f}{unit})",
            )
        return ('ok', '')

    def _build_cartesian_axis_map(self, axis: str, samples: Optional[int] = None,
                                  max_jump_deg: float = 25.0,
                                  elbow_jump_deg: Optional[float] = None):
        """Build a continuity-filtered IK map for a single cartesian axis."""
        if axis not in self._cart_axis_limits:
            return None

        samples = int(samples or self.axis_interp_samples)

        base_pose = [float(v) for v in self.tcp_pose]
        axis_idx = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}[axis]
        lo, hi = self._cart_axis_limits[axis]
        values = np.linspace(float(lo), float(hi), int(samples))
        start_idx = int(np.argmin(np.abs(values - base_pose[axis_idx])))

        current = np.array([float(v) for v in self.current_joints], dtype=float)
        accepted = {start_idx: current.copy()}

        for direction in (1, -1):
            prev = current.copy()
            i = start_idx + direction
            while 0 <= i < len(values):
                pose_i = base_pose.copy()
                pose_i[axis_idx] = float(values[i])
                cand = self._vk_solve_pose(tuple(pose_i))
                if cand is None:
                    break

                # Wrap candidate around previous solution for continuity.
                cand_u = np.array([
                    prev[j] + (((cand[j] - prev[j] + 180.0) % 360.0) - 180.0)
                    for j in range(6)
                ], dtype=float)

                diffs = np.abs(cand_u - prev)
                if float(np.max(diffs)) > float(max_jump_deg):
                    break
                if elbow_jump_deg is not None and (
                    diffs[1] > float(elbow_jump_deg) or diffs[2] > float(elbow_jump_deg)
                ):
                    break

                accepted[i] = cand_u.copy()
                prev = cand_u
                i += direction

        if len(accepted) < 2:
            return None

        idx_sorted = sorted(accepted.keys())
        vals = np.array([values[i] for i in idx_sorted], dtype=float)
        joints = np.vstack([accepted[i] for i in idx_sorted]).astype(float)

        return {
            'axis': axis,
            'axis_idx': axis_idx,
            'values': vals,
            'joints': joints,
            'base_pose': tuple(base_pose),
        }

    def _solve_from_axis_map(self, axis: str, pose: tuple, target_axis_val: float):
        """Interpolate joints from a cached single-axis map if context matches."""
        axis_idx = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}[axis]
        axis_map = self._cart_axis_maps.get(axis)
        if axis_map is None or len(axis_map.get('values', [])) < 2:
            return None

        vals = np.asarray(axis_map['values'], dtype=float)
        joints_mat = np.asarray(axis_map['joints'], dtype=float)

        base_pose = axis_map.get('base_pose', pose)
        mismatch = 0.0
        for i in range(6):
            if i == axis_idx:
                continue
            diff = abs(float(pose[i]) - float(base_pose[i]))
            if i >= 3:
                diff = abs(
                    self._unwrap_angle_near(float(pose[i]), float(base_pose[i]))
                    - float(base_pose[i])
                )
            mismatch = max(mismatch, diff)

        in_range = float(vals[0]) <= float(target_axis_val) <= float(vals[-1])
        if not in_range:
            return None

        allowed_mismatch = 5.0 if axis_idx < 3 else 8.0
        if mismatch > allowed_mismatch:
            return None

        return [
            float(np.interp(float(target_axis_val), vals, joints_mat[:, j]))
            for j in range(6)
        ]

    def _save_axis_storage_maps(self):
        """Persist axis maps to disk for reuse between sessions."""
        try:
            payload = {}
            for axis, amap in self._cart_axis_maps.items():
                payload[axis] = {
                    'values': np.asarray(amap['values'], dtype=float).tolist(),
                    'joints': np.asarray(amap['joints'], dtype=float).tolist(),
                    'base_pose': list(amap.get('base_pose', self.tcp_pose)),
                }
            with open(self.axis_storage_file, "w") as fh:
                json.dump(payload, fh, indent=2)
        except Exception:
            pass

    def _load_axis_storage_maps(self):
        """Load cached maps if available."""
        if not os.path.exists(self.axis_storage_file):
            return
        try:
            with open(self.axis_storage_file, "r") as fh:
                data = json.load(fh)
            for axis, amap in data.items():
                self._cart_axis_maps[axis] = {
                    'axis': axis,
                    'axis_idx': {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}.get(axis, 0),
                    'values': np.asarray(amap.get('values', []), dtype=float),
                    'joints': np.asarray(amap.get('joints', []), dtype=float),
                    'base_pose': tuple(amap.get('base_pose', self.tcp_pose)),
                }
        except Exception:
            pass

    def _refresh_axis_storage_maps(self):
        """Build per-axis IK lookup tables while idle to prevent jog flips."""
        if self._motion_context_active():
            return
        built = {}
        for axis in ('x', 'y', 'z', 'rx', 'ry', 'rz'):
            axis_map = self._build_cartesian_axis_map(
                axis,
                samples=self.axis_storage_samples,
                max_jump_deg=self.axis_storage_max_jump,
                elbow_jump_deg=self.axis_storage_elbow_jump,
            )
            if axis_map is not None:
                built[axis] = axis_map
        if built:
            self._cart_axis_maps.update(built)
            self._save_axis_storage_maps()

    # ===================== MOTION PATH POSE CHECK =======================
    def _check_motion_joints(self, joint_frames: list) -> list:
        """Check & correct a list of joint frames for pose quality.

        Runs the same checks as the trajectory analyser:
        velocity spikes, wrist singularity, joint-limit proximity,
        and applies the same interpolation-based corrections.

        Parameters
        ----------
        joint_frames : list[list[float]]
            Sequence of 6-element joint angle lists (degrees).

        Returns
        -------
        list[list[float]]
            Corrected joint frames (same length as input).
        """
        N = len(joint_frames)
        if N < 3:
            return joint_frames          # too short to analyse

        VEL_SPIKE = 15.0                 # °/frame
        SING_DEG  = 5.0                  # J5 within ±5° of 0°
        JOINT_LIM = 180.0
        LIM_MARG  = 5.0                  # clamp margin

        cj = np.array(joint_frames, dtype=float)   # working copy

        def _regions(mask):
            regs, in_r = [], False
            st = 0
            for i in range(len(mask)):
                if mask[i] and not in_r:
                    st = i; in_r = True
                elif not mask[i] and in_r:
                    regs.append((st, i - 1)); in_r = False
            if in_r:
                regs.append((st, len(mask) - 1))
            return regs

        applied = False

        # ── Velocity spikes (IK flips) ──
        dj = np.abs(np.diff(cj, axis=0))
        spike_mask = np.zeros(N, dtype=bool)
        for i in range(dj.shape[0]):
            if np.any(dj[i] > VEL_SPIKE):
                spike_mask[i + 1] = True
        if np.any(spike_mask):
            for (rs, re) in _regions(spike_mask):
                ab = max(rs - 1, 0)
                aa = min(re + 1, N - 1)
                span = aa - ab
                if span < 1:
                    continue
                for i in range(rs, re + 1):
                    t = (i - ab) / span
                    cj[i] = (1 - t) * cj[ab] + t * cj[aa]
            applied = True

        # ── Wrist singularity (smooth J4/J6) ──
        sing_mask = np.abs(cj[:, 4]) < SING_DEG
        if np.any(sing_mask):
            for (rs, re) in _regions(sing_mask):
                ab = max(rs - 1, 0)
                aa = min(re + 1, N - 1)
                span = aa - ab
                if span < 1:
                    continue
                for jj in (3, 5):
                    vb, va = cj[ab, jj], cj[aa, jj]
                    for i in range(rs, re + 1):
                        t = (i - ab) / span
                        cj[i, jj] = (1 - t) * vb + t * va
            applied = True

        # ── Joint-limit clamp ──
        safe_lim = JOINT_LIM - LIM_MARG
        clamped = np.clip(cj, -safe_lim, safe_lim)
        if not np.array_equal(clamped, cj):
            cj = clamped
            applied = True

        # ── Second-pass residual spikes ──
        dj2 = np.abs(np.diff(cj, axis=0))
        spike2 = np.zeros(N, dtype=bool)
        for i in range(dj2.shape[0]):
            if np.any(dj2[i] > VEL_SPIKE):
                spike2[i + 1] = True
        if np.any(spike2):
            for (rs, re) in _regions(spike2):
                ab = max(rs - 1, 0)
                aa = min(re + 1, N - 1)
                span = aa - ab
                if span < 1:
                    continue
                for i in range(rs, re + 1):
                    t = (i - ab) / span
                    cj[i] = (1 - t) * cj[ab] + t * cj[aa]
            applied = True

        if applied:
            self.log_event(
                "Motion path corrected (singularity / flip / limit)",
                "CORRECTION")

        return [cj[i].tolist() for i in range(N)]

    # =================== TRAPEZOIDAL VELOCITY PROFILE =======================
    @staticmethod
    def _trapezoidal_profile(n_frames, accel_pct=0.20, decel_pct=0.20):
        """Return (n_frames+1) position fractions s ∈ [0,1] following a
        trapezoidal velocity profile: accelerate → cruise → decelerate.

        Parameters
        ----------
        n_frames    : int   – number of intervals (returns n_frames+1 points)
        accel_pct   : float – fraction of total time spent accelerating
        decel_pct   : float – fraction of total time spent decelerating

        The remaining (1 - accel_pct - decel_pct) is cruise at constant speed.
        """
        cruise_pct = 1.0 - accel_pct - decel_pct
        # peak velocity so total area under trapezoid = 1.0
        v_max = 1.0 / (0.5 * accel_pct + cruise_pct + 0.5 * decel_pct)
        a_acc = v_max / accel_pct if accel_pct > 0 else 0.0
        a_dec = v_max / decel_pct if decel_pct > 0 else 0.0
        s_end_accel = 0.5 * v_max * accel_pct
        s_end_cruise = s_end_accel + v_max * cruise_pct

        result = []
        for i in range(n_frames + 1):
            t = i / n_frames
            if t <= accel_pct:
                # accelerating
                s = 0.5 * a_acc * t * t
            elif t <= accel_pct + cruise_pct:
                # cruising at v_max
                s = s_end_accel + v_max * (t - accel_pct)
            else:
                # decelerating
                td = t - accel_pct - cruise_pct
                s = s_end_cruise + v_max * td - 0.5 * a_dec * td * td
            result.append(max(0.0, min(1.0, s)))
        return result

    def _frames_for_target_speed(self, distance: float, speed: float,
                                 min_frames: int = 6, max_frames: int = 300):
        """Compute animation frame count from distance and target speed.

        distance and speed are in consistent units (mm or deg).
        """
        safe_speed = max(0.1, float(speed))
        total_s = max(0.2, float(distance) / safe_speed)
        n_frames = int(np.ceil((total_s * 1000.0) / self.motion_frame_interval_ms))
        return max(min_frames, min(max_frames, n_frames)), total_s

    # ======================= P2P MOTION =======================
    def start_p2p_motion(self, target_joints, target_tcp=None, on_complete=None):
        """Point-to-point motion with trapezoidal velocity profile.
        Accel → cruise → decel in joint space, ~12 visual frames."""
        if self.linear_active:
            self._finish_linear_motion(snap=False)
        if self.p2p_active:
            self._finish_p2p_motion()  # Cancel any running P2P

        self.p2p_start_joints = self.current_joints.copy()
        self.p2p_target_joints = list(target_joints)
        self.p2p_target_tcp = target_tcp
        self.p2p_on_complete = on_complete

        # ── Joint deltas (preserve unwind direction) ──
        # Use raw signed differences so wrist unwind is preserved
        # (e.g. +270 -> 0 follows -270 instead of +90).
        deltas = [
            self.p2p_target_joints[i] - self.p2p_start_joints[i]
            for i in range(6)
        ]

        # Speed-based duration: movement_speed is treated as deg/s in P2P.
        max_delta = max(abs(d) for d in deltas)
        N, total_s = self._frames_for_target_speed(max_delta, self.movement_speed)
        profile = self._trapezoidal_profile(N)  # N+1 values including s=0
        self.p2p_frames = []
        for s in profile[1:]:  # skip s=0 (that's the start position)
            frame = [
                self.p2p_start_joints[i] + s * deltas[i]
                for i in range(6)
            ]
            self.p2p_frames.append(frame)
        self.p2p_current_step = 0
        self.p2p_total_steps = N

        self.p2p_active = True
        self.p2p_timer.start(self.motion_frame_interval_ms)

        # Switch to 3D View tab
        if hasattr(self, 'status_tabs'):
            self.status_tabs.setCurrentIndex(1)

        self.log_event(
            f"P2P → max Δ={max_delta:.1f}°, v={self.movement_speed:.1f}°/s, "
            f"t={total_s:.2f}s, {N} frames",
            "JOG"
        )

    def _p2p_tick(self):
        """One frame of the P2P visual sweep."""
        if not self.p2p_active:
            self.p2p_timer.stop()
            return

        idx = min(self.p2p_current_step, len(self.p2p_frames) - 1)
        frame = self.p2p_frames[idx]
        self.current_joints = list(frame)
        self._sync_unwrapped_wrist()
        self.p2p_current_step += 1

        # Update joint displays
        for i, jv in enumerate(self.current_joints):
            if i < len(self.joint_inputs):
                self.joint_inputs[i].setText(f"{jv:.1f}")
            if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                self.joint_labels[i].setText(f"{jv:.3f}°")

        # Update 3D visualiser
        if self.robot_visualizer:
            self.robot_visualizer.update_joints(self.current_joints)

        # Request FK to update TCP display
        if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
            self.fk_thread.request_fk(self.current_joints)

        if self.p2p_current_step >= self.p2p_total_steps:
            self._finish_p2p_motion()

    def _finish_p2p_motion(self):
        """Snap to exact target and fire callback."""
        self.p2p_timer.stop()
        was_active = self.p2p_active
        self.p2p_active = False

        if was_active and self.p2p_target_joints:
            # Snap to exact target joints
            self.current_joints = list(self.p2p_target_joints)
            self._sync_unwrapped_wrist()
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_inputs):
                    self.joint_inputs[i].setText(f"{jv:.1f}")
                if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                    self.joint_labels[i].setText(f"{jv:.3f}°")

            if self.p2p_target_tcp:
                self.tcp_pose = self._canonicalize_tcp_pose(self.p2p_target_tcp)
                self.schedule_cartesian_display_update()
                x, y, z, rx, ry, rz = self.tcp_pose
                self.tcp_labels['x'].setText(f"{x:.2f}")
                self.tcp_labels['y'].setText(f"{y:.2f}")
                self.tcp_labels['z'].setText(f"{z:.2f}")
                self.tcp_labels['rx'].setText(f"{rx:.2f}")
                self.tcp_labels['ry'].setText(f"{ry:.2f}")
                self.tcp_labels['rz'].setText(f"{rz:.2f}")

            if self.robot_visualizer:
                self.robot_visualizer.update_joints(self.current_joints)

            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self.fk_thread.request_fk(self.current_joints)

            self.log_event("P2P complete", "SUCCESS")
            self.save_jog_data()

            cb = self.p2p_on_complete
            if callable(cb):
                cb()

    # ===================== TRANSITION MOTION DISPATCHER =====================
    def start_transition_motion(self, target_joints, target_tcp=None, on_complete=None):
        """Dispatch to P2P or Linear based on self.transition_mode.
        Falls back to P2P when target_tcp is unavailable (Linear needs it)."""
        if self.transition_mode == "LINEAR" and target_tcp is not None:
            # Need current TCP accurately — compute FK synchronously
            start_tcp = tuple(self.tcp_pose)
            # If start TCP looks uninitialised, compute it from joints
            if all(v == 0.0 for v in start_tcp):
                try:
                    start_tcp = solve_fk_gui(self.current_joints)
                except Exception:
                    pass
            self.start_linear_motion(start_tcp, target_tcp, target_joints, on_complete)
        else:
            if self.transition_mode == "LINEAR" and target_tcp is None:
                self.log_event("Linear mode: no TCP data — falling back to P2P", "WARNING")
            self.start_p2p_motion(target_joints, target_tcp, on_complete)

    def generate_stable_joint_path(self, cartesian_points, initial_joint_state):
        """Generate a continuous joint trajectory from Cartesian waypoints.

        Rules enforced:
        - Seeded IK continuity via previous joint state
        - Branch-consistent candidate selection (prefer same elbow branch)
        - Hard per-step jump rejection (>10 deg on any joint)
        - Key-waypoint reduction before IK
        - Joint-space interpolation after waypoint IK
        - Optional light moving-average smoothing
        """
        if not cartesian_points:
            return []

        def _wrapped_diff(new_v, cur_v):
            return ((float(new_v) - float(cur_v) + 180.0) % 360.0) - 180.0

        def _closest_unwrapped(new_v, cur_v):
            return float(cur_v) + _wrapped_diff(new_v, cur_v)

        def _branch_id(joints):
            # Elbow branch proxy from J3 sign; stable and cheap.
            j3 = float(joints[2])
            return 1 if j3 >= 0.0 else -1

        # 1) Reduce Cartesian density: keep only key waypoints.
        raw_points = [tuple(float(v) for v in p[:6]) for p in cartesian_points]
        key_points = [raw_points[0]]
        min_dist_mm = 8.0
        min_ori_deg = 2.0
        for p in raw_points[1:-1]:
            lp = key_points[-1]
            dpos = float(np.linalg.norm(np.array(p[:3]) - np.array(lp[:3])))
            dori = max(abs(_wrapped_diff(p[3 + i], lp[3 + i])) for i in range(3))
            if dpos >= min_dist_mm or dori >= min_ori_deg:
                key_points.append(p)
        if len(raw_points) > 1:
            key_points.append(raw_points[-1])

        # Cap waypoint count for real-time use while preserving endpoints.
        max_waypoints = 40
        if len(key_points) > max_waypoints:
            idx = np.linspace(0, len(key_points) - 1, max_waypoints, dtype=int)
            key_points = [key_points[i] for i in idx]

        prev = np.array([float(v) for v in initial_joint_state], dtype=float)
        prev_branch = _branch_id(prev)
        waypoint_joints = [prev.tolist()]
        rejected_jumps = 0
        branch_switches = 0

        # 2) Solve IK per key waypoint, select nearest seeded candidate.
        for pose in key_points[1:]:
            x, y, z, rx, ry, rz = pose
            candidates = []

            # Core 3R candidates (keep wrist near seed).
            for j1, j2, j3 in self._solve_3r_position_yaskawa(x, y, z):
                c = prev.copy()
                c[0], c[1], c[2] = float(j1), float(j2), float(j3)
                candidates.append(c)

            # VK candidates (both conventions).
            sol = solve_ik_gui((x, y, z, rx, ry, rz), seed=prev.tolist())
            if sol is not None:
                candidates.append(np.array([float(v) for v in sol], dtype=float))

            # De-duplicate near-identical candidates.
            uniq = []
            seen = set()
            for c in candidates:
                key = tuple(int(round(float(v) * 100.0)) for v in c)
                if key in seen:
                    continue
                seen.add(key)
                uniq.append(c)
            candidates = uniq

            if not candidates:
                # No IK: hold previous state to preserve continuity.
                waypoint_joints.append(prev.tolist())
                continue

            scored = []
            for c in candidates:
                # Compare in shortest-angle sense and unwrap around prev.
                cu = np.array([_closest_unwrapped(c[i], prev[i]) for i in range(6)], dtype=float)
                d = np.abs(cu - prev)
                max_step = float(np.max(d))
                sse = float(np.sum((cu - prev) ** 2))
                scored.append((cu, d, max_step, sse, _branch_id(cu)))

            # 3) Reject candidates with >10 deg on any joint.
            valid = [row for row in scored if row[2] <= 10.0]

            # 4) Keep branch consistency if possible.
            same_branch_valid = [row for row in valid if row[4] == prev_branch]
            if same_branch_valid:
                chosen = min(same_branch_valid, key=lambda row: row[3])
            elif valid:
                chosen = min(valid, key=lambda row: row[3])
                branch_switches += 1
            else:
                # No valid low-jump move: hold previous to avoid discontinuity.
                rejected_jumps += 1
                waypoint_joints.append(prev.tolist())
                continue

            prev = chosen[0]
            prev_branch = chosen[4]
            waypoint_joints.append(prev.tolist())

        # 6) Interpolate in JOINT space between stable waypoint solutions.
        dense = [waypoint_joints[0]]
        max_joint_step_deg = 2.0
        for i in range(1, len(waypoint_joints)):
            q0 = np.array(dense[-1], dtype=float)
            q1 = np.array(waypoint_joints[i], dtype=float)
            max_d = float(np.max(np.abs(q1 - q0)))
            steps = max(1, int(np.ceil(max_d / max_joint_step_deg)))
            for k in range(1, steps + 1):
                t = k / steps
                q = (1.0 - t) * q0 + t * q1
                dense.append(q.tolist())

        # 7) Optional smoothing (light moving average).
        smooth_window = 3
        if smooth_window > 1 and len(dense) >= smooth_window:
            arr = np.array(dense, dtype=float)
            out = arr.copy()
            half = smooth_window // 2
            for i in range(1, len(arr) - 1):
                a = max(0, i - half)
                b = min(len(arr), i + half + 1)
                out[i] = np.mean(arr[a:b], axis=0)
            dense = [out[i].tolist() for i in range(len(out))]

        if rejected_jumps:
            self.log_event(
                f"Stable IK: held previous state at {rejected_jumps} waypoint(s) due to >10deg jump",
                "WARNING",
            )
        if branch_switches:
            self.log_event(
                f"Stable IK: branch switched {branch_switches} time(s) (no same-branch valid solution)",
                "WARNING",
            )

        return dense

    # ====================== LINEAR (CARTESIAN) MOTION ======================
    def start_linear_motion(self, start_tcp, target_tcp, target_joints, on_complete=None):
        """Straight-line TCP motion with sparse waypoints.
        Uses max 5 waypoints per 200 mm of travel.  IK is solved at each
        waypoint to verify reachability.  The final joint angles confirm
        the target pose (99.9% accurate)."""
        # Cancel any running motion
        if self.p2p_active:
            self._finish_p2p_motion()
        if self.linear_active:
            self._finish_linear_motion()

        self.linear_start_tcp = start_tcp
        self.linear_target_tcp = target_tcp
        self.linear_target_joints = list(target_joints)
        self.linear_on_complete = on_complete

        # ── Unpack start / end ──
        sx, sy, sz, srx, sry, srz = start_tcp
        tx, ty, tz, trx, try_, trz = target_tcp

        dist = np.sqrt((tx - sx)**2 + (ty - sy)**2 + (tz - sz)**2)
        ori_delta_deg = self._quat_orientation_distance_deg(
            (srx, sry, srz),
            (trx, try_, trz),
        )
        pos_waypoints = max(2, min(20, int(np.ceil(dist / 30.0))))
        ori_waypoints = max(2, min(20, int(np.ceil(ori_delta_deg / 8.0))))
        n_waypoints = max(pos_waypoints, ori_waypoints)
        waypoint_t = [step / n_waypoints for step in range(n_waypoints + 1)]
        waypoint_orientations = self._sample_orientation_slerp(
            (srx, sry, srz),
            (trx, try_, trz),
            waypoint_t,
        )

        waypoint_tcp = []
        for step, t in enumerate(waypoint_t):
            orx, ory, orz = waypoint_orientations[step]
            waypoint_tcp.append((
                sx + t * (tx - sx),
                sy + t * (ty - sy),
                sz + t * (tz - sz),
                orx,
                ory,
                orz,
            ))

        # VK baseline: solve IK on sparse waypoints, then interpolate in
        # joint space between solved waypoints.
        vk_waypoint_joints = []
        prev_seed = self.current_joints
        for i, (px, py, pz, prx, pry, prz) in enumerate(waypoint_tcp):
            ik = solve_ik_gui((px, py, pz, prx, pry, prz), seed=prev_seed)
            if ik is None:
                self.log_event(
                    f"Linear motion aborted: VK IK failed at waypoint {i + 1}/{len(waypoint_tcp)}",
                    "ERROR"
                )
                return
            vk_waypoint_joints.append(np.array([float(v) for v in ik], dtype=float))
            prev_seed = list(ik)

        stable_frames = [vk_waypoint_joints[0].tolist()]
        max_joint_step_deg = 2.0
        for i in range(1, len(vk_waypoint_joints)):
            q0 = np.array(stable_frames[-1], dtype=float)
            q1 = vk_waypoint_joints[i]
            max_d = float(np.max(np.abs(q1 - q0)))
            steps = max(1, int(np.ceil(max_d / max_joint_step_deg)))
            for k in range(1, steps + 1):
                t = k / steps
                q = (1.0 - t) * q0 + t * q1
                stable_frames.append(q.tolist())

        # Match TCP display points to generated joint-frame count.
        visual_tcp = []
        dense_t = [i / max(1, (len(stable_frames) - 1)) for i in range(len(stable_frames))]
        dense_orientations = self._sample_orientation_slerp(
            (srx, sry, srz),
            (trx, try_, trz),
            dense_t,
        )
        for i, t in enumerate(dense_t):
            orx, ory, orz = dense_orientations[i]
            visual_tcp.append((
                sx + t * (tx - sx),
                sy + t * (ty - sy),
                sz + t * (tz - sz),
                orx,
                ory,
                orz,
            ))

        self.linear_corrected_frames = stable_frames
        self.linear_tcp_path = visual_tcp
        self.linear_total_steps = max(1, len(stable_frames) - 1)
        self.linear_current_step = 0

        self.linear_active = True
        self.linear_timer.start(self.motion_frame_interval_ms)

        if hasattr(self, 'status_tabs'):
            self.status_tabs.setCurrentIndex(1)

        self.log_event(
            f"Linear → {dist:.0f} mm, ori {ori_delta_deg:.1f}°, v={self.movement_speed:.1f} mm/s, "
            f"{n_waypoints} sparse IK pts, {len(stable_frames)} joint frames",
            "JOG"
        )

    def _linear_tick(self):
        """One step of Linear motion (uses pre-computed corrected frames)."""
        if not self.linear_active:
            self.linear_timer.stop()
            return

        self.linear_current_step += 1
        idx = min(self.linear_current_step,
                  len(self.linear_corrected_frames) - 1)
        joints = self.linear_corrected_frames[idx]
        self.current_joints = list(joints)
        self._sync_unwrapped_wrist()

        # Retrieve the matching TCP from pre-computed path
        tcp = self.linear_tcp_path[idx]
        ix, iy, iz, irx, iry, irz = tcp
        self.tcp_pose = self._canonicalize_tcp_pose(tcp)
        ix, iy, iz, irx, iry, irz = self.tcp_pose

        # ── Update displays ──
        for i, jv in enumerate(joints):
            if i < len(self.joint_inputs):
                self.joint_inputs[i].setText(f"{jv:.1f}")
            if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                self.joint_labels[i].setText(f"{jv:.3f}°")

        self.tcp_labels['x'].setText(f"{ix:.2f}")
        self.tcp_labels['y'].setText(f"{iy:.2f}")
        self.tcp_labels['z'].setText(f"{iz:.2f}")
        self.tcp_labels['rx'].setText(f"{irx:.2f}")
        self.tcp_labels['ry'].setText(f"{iry:.2f}")
        self.tcp_labels['rz'].setText(f"{irz:.2f}")
        if hasattr(self, 'cartesian_inputs'):
            self.schedule_cartesian_display_update()

        if self.robot_visualizer:
            self.robot_visualizer.update_joints(joints)

        if self.linear_current_step >= self.linear_total_steps:
            self._finish_linear_motion(snap=True)

    def _finish_linear_motion(self, snap=True):
        """Complete or abort Linear motion."""
        self.linear_timer.stop()
        was_active = self.linear_active
        self.linear_active = False
        self.linear_slerp = None

        if was_active and snap and self.linear_target_joints:
            # Snap to exact target
            self.current_joints = list(self.linear_target_joints)
            self._sync_unwrapped_wrist()
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_inputs):
                    self._set_text_if_changed(self.joint_inputs[i], f"{jv:.1f}")
                if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                    self._set_text_if_changed(self.joint_labels[i], f"{jv:.3f}°")

            if self.linear_target_tcp:
                self.tcp_pose = self._canonicalize_tcp_pose(self.linear_target_tcp)
                x, y, z, rx, ry, rz = self.tcp_pose
                self._set_text_if_changed(self.tcp_labels['x'], f"{x:.2f}")
                self._set_text_if_changed(self.tcp_labels['y'], f"{y:.2f}")
                self._set_text_if_changed(self.tcp_labels['z'], f"{z:.2f}")
                self._set_text_if_changed(self.tcp_labels['rx'], f"{rx:.2f}")
                self._set_text_if_changed(self.tcp_labels['ry'], f"{ry:.2f}")
                self._set_text_if_changed(self.tcp_labels['rz'], f"{rz:.2f}")
                if hasattr(self, 'cartesian_inputs'):
                    self.schedule_cartesian_display_update()

            if self.robot_visualizer:
                self.robot_visualizer.update_joints(self.current_joints)

            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self.fk_thread.request_fk(self.current_joints)

            self.log_event("Linear motion completed", "SUCCESS")
            self.save_jog_data()

            # Fire optional callback
            cb = self.linear_on_complete
            if callable(cb):
                cb()
        elif was_active and not snap:
            self.log_event("Linear motion stopped (IK failure)", "WARNING")
            self.save_jog_data()

    # =============== TRAJECTORY PLAYBACK SIGNAL HANDLERS ===================
    @pyqtSlot(int, list)
    def _on_traj_frame_changed(self, frame_idx, joints):
        """Keep MainWindow state in sync during trajectory playback.
        Updates joints immediately and requests FK for cartesian."""
        self.current_joints = joints.copy()

        # ── Immediate joint display updates (no FK needed) ──
        for i, jv in enumerate(joints):
            if hasattr(self, 'joint_labels') and i < len(self.joint_labels):
                self._set_text_if_changed(self.joint_labels[i], f"{jv:.3f}°")
            if i < len(self.joint_inputs):
                self._set_text_if_changed(self.joint_inputs[i], f"{jv:.1f}")

        # ── If trajectory has Cartesian data, use it directly for TCP
        #    (faster & more accurate than re-computing FK every frame) ──
        viz = self.robot_visualizer
        if (viz and viz.traj_has_cartesian
                and frame_idx < len(viz.traj_cartesian)):
            cd = viz.traj_cartesian[frame_idx]
            if self._traj_weld_sync_active and 'weld_on' in cd:
                self._set_weld_state_for_playback(bool(cd.get('weld_on', 0)))
            pos = cd['position']      # mm
            ori = cd['orientation']    # degrees
            self.tcp_pose = self._canonicalize_tcp_pose((pos[0], pos[1], pos[2], ori[0], ori[1], ori[2]))
            x, y, z, rx, ry, rz = self.tcp_pose
            self._set_text_if_changed(self.tcp_labels['x'], f"{x:.2f}")
            self._set_text_if_changed(self.tcp_labels['y'], f"{y:.2f}")
            self._set_text_if_changed(self.tcp_labels['z'], f"{z:.2f}")
            self._set_text_if_changed(self.tcp_labels['rx'], f"{rx:.2f}")
            self._set_text_if_changed(self.tcp_labels['ry'], f"{ry:.2f}")
            self._set_text_if_changed(self.tcp_labels['rz'], f"{rz:.2f}")
            if hasattr(self, 'cartesian_inputs'):
                self.schedule_cartesian_display_update()
        else:
            # Fallback: request FK to compute TCP from joints
            if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
                self.fk_thread.request_fk(joints)

    @pyqtSlot()
    def _on_traj_finished(self):
        """Handle trajectory playback completion."""
        self._restore_weld_state_after_playback()
        try:
            self.save_jog_data()
        except Exception as exc:
            self.log_event(f"Failed to save final playback pose: {exc}", "WARNING")
        self.log_event("Trajectory playback finished", "SUCCESS")

    @pyqtSlot(tuple)
    def on_fk_ready(self, pose):
        """Handle FK calculation results — updates ALL displays unconditionally."""
        new_pose = self._display_pose_from_fk(pose)
        self.tcp_pose = new_pose
        self.last_fk_joints = self.current_joints.copy()

        # ── Status-panel TCP labels (always) ──
        self._update_tcp_labels_from_pose(self.tcp_pose)

        # ── Status-panel joint labels (always) ──
        if hasattr(self, 'joint_labels'):
            for i, jv in enumerate(self.current_joints):
                if i < len(self.joint_labels):
                    self._set_text_if_changed(self.joint_labels[i], f"{jv:.3f}°")

        # ── 3D visualiser (skips internally when its own playback is driving) ──
        if hasattr(self, 'robot_visualizer') and self.robot_visualizer is not None:
            self.robot_visualizer.update_joints(self.current_joints)

        # ── Cartesian jog inputs – ALWAYS keep in sync (user isn't typing
        #    cartesian values while joints move; the old gate was preventing
        #    updates entirely) ──
        if hasattr(self, 'cartesian_inputs'):
            self.schedule_cartesian_display_update()
        
    @pyqtSlot(list)
    def on_ik_ready(self, joints):
        """Handle IK calculation results — updates ALL displays unconditionally."""
        self._pending_cartesian_fk_lock = None
        self.current_joints = joints.copy()

        # ── Status-panel joint labels (always) ──
        if hasattr(self, 'joint_labels'):
            for i, jv in enumerate(joints):
                if i < len(self.joint_labels):
                    self._set_text_if_changed(self.joint_labels[i], f"{jv:.3f}°")

        # ── 3D visualiser ──
        if hasattr(self, 'robot_visualizer') and self.robot_visualizer is not None:
            self.robot_visualizer.update_joints(joints)

        # ── Joint jog inputs – always keep in sync ──
        if hasattr(self, 'joint_inputs'):
            for i, jv in enumerate(joints):
                if i < len(self.joint_inputs):
                    self._set_text_if_changed(self.joint_inputs[i], f"{jv:.2f}")

        # ── Request FK to refresh TCP / cartesian fields ──
        if hasattr(self, 'fk_thread') and self.fk_thread.isRunning():
            self.fk_thread.request_fk(joints)
        
    @pyqtSlot(str)
    def on_fk_error(self, error_msg):
        """Handle FK and IK calculation errors"""
        print(f"Kinematic Error: {error_msg}")
        self.status_bar.showMessage(f"Kinematic Error: {error_msg}", 5000)
        
    def update_robot_state(self):
        """Update robot state simulation"""
        # In a real application, this would poll actual robot hardware
        # For now, we simulate some activity
        pass
        
    def closeEvent(self, event):
        """Clean up when closing the application"""
        # ── 0. Unsaved-changes guard ──────────────────────────
        if getattr(self, 'program_modified', False):
            from PyQt6.QtWidgets import QMessageBox
            reply = QMessageBox.question(
                self, "Unsaved Changes",
                "You have unsaved changes.\nSave before closing?",
                QMessageBox.StandardButton.Save
                | QMessageBox.StandardButton.Discard
                | QMessageBox.StandardButton.Cancel,
            )
            if reply == QMessageBox.StandardButton.Cancel:
                event.ignore()
                return
            if reply == QMessageBox.StandardButton.Save:
                self.save_program()
        # ──────────────────────────────────────────────────────
        print("Application closing...")
        
        # Flush any pending batched console messages before exit
        if hasattr(self, '_console_batch_timer') and self._console_batch_timer.isActive():
            self._console_batch_timer.stop()
            self._flush_console_batch()
        
        # ── 1. Stop every QTimer so nothing fires during teardown ──
        for attr in ('robot_timer', 'movement_timeout_timer', 'cartesian_update_timer', 'p2p_timer', 'linear_timer', 'jog_preplan_timer'):
            t = getattr(self, attr, None)
            if t is not None:
                t.stop()
        for t in getattr(self, 'jog_timers', {}).values():
            if t is not None:
                t.stop()
        
        # ── 2. Stop the FK worker thread ──
        try:
            if hasattr(self, 'fk_thread') and self.fk_thread:
                self.fk_thread.stop_worker()
                if not self.fk_thread.wait(1000):
                    self.fk_thread.terminate()
                    self.fk_thread.wait(500)
        except Exception:
            pass

        # ── 2a. Stop the jog preplanner thread ──
        try:
            jog_preplan_thread = getattr(self, 'jog_preplan_thread', None)
            if jog_preplan_thread is not None:
                jog_preplan_thread.requestInterruption()
                if not jog_preplan_thread.wait(1000):
                    jog_preplan_thread.terminate()
                    jog_preplan_thread.wait(500)
        except Exception:
            pass
        
        # ── 2b. Stop control, hardware, and pendant threads ──
        try:
            control_thread = getattr(self, 'control_thread', None)
            if control_thread is not None:
                control_thread.stop_gracefully()
                if not control_thread.wait(1000):
                    control_thread.terminate()
                    control_thread.wait(500)
        except Exception:
            pass
        
        try:
            hardware_thread = getattr(self, 'hardware_thread', None)
            if hardware_thread is not None:
                hardware_thread.stop_gracefully()
                if not hardware_thread.wait(1000):
                    hardware_thread.terminate()
                    hardware_thread.wait(500)
        except Exception:
            pass
        
        try:
            pendant_thread = getattr(self, 'pendant_thread', None)
            if pendant_thread is not None:
                pendant_thread.running = False
                if not pendant_thread.wait(1000):
                    pendant_thread.terminate()
                    pendant_thread.wait(500)
        except Exception:
            pass
        
        # ── 3. Stop the 3D visualiser timers before widget teardown ──
        viz = getattr(self, 'robot_visualizer', None)
        if viz is not None:
            viz.cleanup()
        
        # ── 4. Save user data ──
        try:
            self.save_jog_data()
        except Exception:
            pass
        
        # ── 5. Flush session log to file ──
        try:
            self._flush_log_to_file()
        except Exception:
            pass
        
        print("Cleanup done")
        event.accept()


def main():
    """Main application entry point"""
    # Set Qt application attributes before creating QApplication
    QApplication.setAttribute(Qt.ApplicationAttribute.AA_DontCreateNativeWidgetSiblings, True)

    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
