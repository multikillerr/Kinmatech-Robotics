#!/usr/bin/env python3
"""
Kinematic Worker Thread for Forward and Inverse Kinematics Calculations
Handles FK/IK calculations in a separate thread to prevent UI blocking
"""

import sys
import os
import time
from typing import List, Optional, Tuple

# Ensure project root is in path so sibling layer modules can be found
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from PyQt6.QtCore import QThread, QObject, pyqtSignal, QMutex, QMutexLocker
from PyQt6.QtWidgets import QApplication
from planning.kinematics_adapter import solve_fk_gui, solve_ik_gui


class KinematicWorker(QObject):
    """Worker class for kinematic calculations running in separate thread"""
    
    # Signals
    fk_ready = pyqtSignal(tuple)  # GUI pose: (x, y, z, rx, ry, rz)
    ik_ready = pyqtSignal(list)   # [j1, j2, j3, j4, j5, j6]
    error = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self._mutex = QMutex()
        self._pending_joints: Optional[List[float]] = None
        self._pending_pose: Optional[Tuple[float, float, float, float, float, float]] = None
        self._running = True
        self._last_joints: Optional[List[float]] = None
        
    def request_fk(self, joints: List[float]) -> None:
        """Request forward kinematics calculation. Overwrites any pending request."""
        with QMutexLocker(self._mutex):
            self._pending_joints = joints.copy()
    
    def request_ik(self, pose: Tuple[float, float, float, float, float, float]) -> None:
        """Request inverse kinematics calculation. Overwrites any pending request."""
        with QMutexLocker(self._mutex):
            self._pending_pose = pose
    
    def stop(self) -> None:
        """Stop the worker thread"""
        with QMutexLocker(self._mutex):
            self._running = False
    
    def run(self) -> None:
        """Main worker loop - processes FK and IK requests"""
        while True:
            # Check if we should stop
            with QMutexLocker(self._mutex):
                if not self._running:
                    break
                current_joints = self._pending_joints
                current_pose = self._pending_pose
                self._pending_joints = None
                self._pending_pose = None
            
            # Process FK request if we have one
            if current_joints is not None:
                try:
                    pose = self._calculate_forward_kinematics(current_joints)
                    self.fk_ready.emit(pose)
                    self._last_joints = current_joints
                except Exception as e:
                    self.error.emit(f"FK calculation error: {str(e)}")
            
            # Process IK request if we have one
            if current_pose is not None:
                try:
                    joints = self._calculate_inverse_kinematics(current_pose)
                    if joints is not None:
                        self.ik_ready.emit(joints)
                    else:
                        self.error.emit("IK calculation failed: No solution found")
                except Exception as e:
                    self.error.emit(f"IK calculation error: {str(e)}")
            
            # Small delay to prevent busy waiting
            time.sleep(0.01)  # 100 Hz update rate
    
    def _calculate_forward_kinematics(self, joints: List[float]) -> Tuple[float, float, float, float, float, float]:
        """Calculate forward kinematics in GUI pose order."""
        if len(joints) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joints)}")
        x, y, z, rx, ry, rz = solve_fk_gui(joints)
        return (float(x), float(y), float(z), float(rx), float(ry), float(rz))
    
    def _calculate_inverse_kinematics(self, pose: Tuple[float, float, float, float, float, float]) -> Optional[List[float]]:
        """Calculate inverse kinematics from a GUI-order pose."""
        result = solve_ik_gui(pose)
        if result is None:
            return None
        return [float(v) for v in result]


class KinematicThread(QThread):
    """Thread wrapper for the kinematic worker"""
    
    # Forward signals from worker
    fk_ready = pyqtSignal(tuple)
    ik_ready = pyqtSignal(list)
    error = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self._buffered_fk_requests = []  # Buffer for FK requests made before worker exists
        self._buffered_ik_requests = []  # Buffer for IK requests made before worker exists
        self._mutex = QMutex()
        
    def run(self):
        """Run the worker in this thread"""
        self.worker = KinematicWorker()
        
        # Connect worker signals to thread signals
        self.worker.fk_ready.connect(self.fk_ready.emit)
        self.worker.ik_ready.connect(self.ik_ready.emit)
        self.worker.error.connect(self.error.emit)
        
        # Process any buffered requests
        with QMutexLocker(self._mutex):
            for joints in self._buffered_fk_requests:
                self.worker.request_fk(joints)
            for pose in self._buffered_ik_requests:
                self.worker.request_ik(pose)
            self._buffered_fk_requests.clear()
            self._buffered_ik_requests.clear()
        
        # Start the worker
        self.worker.run()
    
    def request_fk(self, joints: List[float]) -> None:
        """Request FK calculation"""
        if self.worker:
            self.worker.request_fk(joints)
        else:
            # Buffer the request if worker doesn't exist yet
            with QMutexLocker(self._mutex):
                self._buffered_fk_requests.clear()  # Only keep latest request
                self._buffered_fk_requests.append(joints.copy())
    
    def request_ik(self, pose: Tuple[float, float, float, float, float, float]) -> None:
        """Request IK calculation"""
        if self.worker:
            self.worker.request_ik(pose)
        else:
            # Buffer the request if worker doesn't exist yet
            with QMutexLocker(self._mutex):
                self._buffered_ik_requests.clear()  # Only keep latest request
                self._buffered_ik_requests.append(pose)
    
    def stop_worker(self) -> None:
        """Stop the worker thread"""
        if self.worker:
            self.worker.stop()
        self.quit()


def joints_changed_significantly(joints1: Optional[List[float]], 
                                joints2: List[float], 
                                epsilon: float = 0.1) -> bool:
    """
    Check if joints have changed more than epsilon degrees
    
    Args:
        joints1: Previous joint angles (None if no previous)
        joints2: Current joint angles  
        epsilon: Minimum change threshold in degrees
        
    Returns:
        True if joints changed significantly or joints1 is None
    """
    if joints1 is None or len(joints1) != len(joints2):
        return True
    
    for j1, j2 in zip(joints1, joints2):
        if abs(j1 - j2) > epsilon:
            return True
    
    return False
