#!/usr/bin/env python3
"""Hardware-layer robot communication interfaces.

UI and control code should depend on these interfaces rather than direct
serial/UART access.
"""

from __future__ import annotations

import glob
import time
from abc import ABC, abstractmethod
from typing import Optional, Sequence

import serial


class RobotHardwareInterface(ABC):
    """Abstract robot hardware interface."""

    @abstractmethod
    def connect(self) -> bool:
        """Establish the underlying hardware connection."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close the underlying hardware connection."""

    @abstractmethod
    def send_joint_angles(self, joint_angles: Sequence[float]) -> bool:
        """Send the next joint-angle target to the robot."""

    @abstractmethod
    def read_feedback(self) -> dict:
        """Read current robot feedback state."""


class MockRobotHardwareInterface(RobotHardwareInterface):
    """In-memory robot interface used until a concrete wire protocol exists."""

    def __init__(self) -> None:
        self.connected = False
        self.current_joints = [0.0] * 6
        self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_state = "Unlocked"
        self.tool_state = "Unlocked"

    def connect(self) -> bool:
        self.connected = True
        return True

    def disconnect(self) -> None:
        self.connected = False

    def send_joint_angles(self, joint_angles: Sequence[float]) -> bool:
        if not self.connected:
            return False
        self.current_joints = [float(value) for value in joint_angles[:6]]
        return True

    def read_feedback(self) -> dict:
        return {
            "joints": self.current_joints.copy(),
            "pose": self.current_pose.copy(),
            "gripper": self.gripper_state,
            "tool": self.tool_state,
            "timestamp": time.time(),
        }


class SerialRobotHardwareInterface(RobotHardwareInterface):
    """Serial/UART robot interface.

    This class keeps all UART access inside the hardware layer. The exact
    wire protocol is conservative: outgoing joint targets are emitted as a
    comma-separated ASCII line and incoming feedback accepts either CSV or a
    blank/invalid line fallback.
    """

    def __init__(self, port: Optional[str] = None, baud_rate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self._serial_port: Optional[serial.Serial] = None
        self._last_feedback = {
            "joints": [0.0] * 6,
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper": "Unlocked",
            "tool": "Unlocked",
            "timestamp": time.time(),
        }

    def connect(self) -> bool:
        if self._serial_port is not None and self._serial_port.is_open:
            return True

        candidate_ports = [self.port] if self.port else (
            glob.glob('/dev/tty.usb*')
            + glob.glob('/dev/ttyUSB*')
            + glob.glob('/dev/ttyACM*')
            + glob.glob('COM*')
        )

        for candidate in candidate_ports:
            if not candidate:
                continue
            try:
                self._serial_port = serial.Serial(candidate, self.baud_rate, timeout=self.timeout)
                self.port = candidate
                return True
            except Exception:
                self._serial_port = None
        return False

    def disconnect(self) -> None:
        if self._serial_port is not None:
            try:
                self._serial_port.close()
            except Exception:
                pass
            self._serial_port = None

    def send_joint_angles(self, joint_angles: Sequence[float]) -> bool:
        if self._serial_port is None:
            return False
        try:
            payload = ','.join(f'{float(value):.3f}' for value in joint_angles[:6]) + '\n'
            self._serial_port.write(payload.encode('ascii', errors='ignore'))
            return True
        except Exception:
            self.disconnect()
            return False

    def read_feedback(self) -> dict:
        if self._serial_port is None:
            return self._last_feedback.copy()

        try:
            if self._serial_port.in_waiting:
                raw_line = self._serial_port.readline().decode(errors='replace').strip()
                parsed = self._parse_feedback_line(raw_line)
                if parsed is not None:
                    self._last_feedback = parsed
        except Exception:
            self.disconnect()
        return self._last_feedback.copy()

    def _parse_feedback_line(self, raw_line: str) -> Optional[dict]:
        if not raw_line:
            return None
        try:
            parts = [float(part.strip()) for part in raw_line.split(',')[:6]]
            if len(parts) != 6:
                return None
            return {
                "joints": parts,
                "pose": self._last_feedback["pose"],
                "gripper": self._last_feedback["gripper"],
                "tool": self._last_feedback["tool"],
                "timestamp": time.time(),
            }
        except Exception:
            return None
