#!/usr/bin/env python3
"""Dedicated motor commander thread.

Phase 1 goal:
- Isolate hardware command/feedback I/O in its own thread.
- Preserve existing HardwareThread/UI signal contracts.
- Keep implementation transport-agnostic via RobotHardwareInterface.

This file intentionally avoids motor-profile specifics for now. Register-level
profiles (400/750 vs 140) will be added in later phases behind this seam.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt6.QtCore import QThread

from hardware.robot_interface import RobotHardwareInterface


class DriveProfile(str, Enum):
    """Drive protocol/profile families known to commander."""

    P400_750 = "400_750_profile"
    P140 = "140_profile"


class OperatingMode(str, Enum):
    """High-level operating mode for command semantics."""

    POSITION = "position"
    VELOCITY = "velocity"


@dataclass(frozen=True)
class JointDriveSpec:
    """Static per-joint drive configuration."""

    joint_index: int
    slave_id: int
    profile: DriveProfile
    power_w: int
    bus_name: str = "bus_a"


@dataclass
class CommanderConfig:
    """Runtime tuning knobs for commander loop behavior."""

    loop_hz: float = 50.0
    reconnect_backoff_s: float = 1.0
    # Confirmed mapping: J1..J6 -> slave IDs 1..6
    joint_slave_ids: Tuple[int, int, int, int, int, int] = (1, 2, 3, 4, 5, 6)
    # Optional override for per-joint drive metadata.
    joint_drive_specs: Optional[Tuple[JointDriveSpec, ...]] = None
    # Reference speed for manual joint jog velocity mode.
    default_joint_jog_speed_deg_s: float = 40.0
    # Serial port paths for the two USB-RS485 buses.
    # bus_a: J1/J2/J3 (400W × 2, 750W × 1) — pymodbus profile
    # bus_b: J4/J5/J6 (140W × 3)            — minimalmodbus profile
    bus_port_a: str = "/dev/tty.usbserial-A"  # override with actual device path
    bus_port_b: str = "/dev/tty.usbserial-B"  # override with actual device path
    bus_baud: int = 115200
    # Safety gate: when True, commander only builds packets and does not
    # perform protocol-level hardware writes from these adapters.
    dry_run_transport: bool = True

    # ── Gearing & encoder resolution ─────────────────────────────────────────
    # Per-joint mechanical gear ratios (output turns / motor turns).
    # Effective encoder counts/degree = (encoder_ppr * gear_ratio) / 360.
    # All values are PLACEHOLDERS — calibrate before enabling live transport.
    joint_gear_ratios: Tuple[float, float, float, float, float, float] = (
        110.0,   # J1 — placeholder
        81.0,   # J2 — placeholder
        45.0,   # J3 — placeholder
        41.0,   # J4 — placeholder
        36.0,   # J5 — placeholder
        25.0,   # J6 — placeholder
    )
    # Encoder pulses per motor revolution for each drive family.
    # 400W/750W (bus_a, pymodbus):       set to actual drive configuration
    # 140W       (bus_b, minimalmodbus): set to actual drive configuration
    encoder_ppr_400_750: int = 10000  # placeholder
    encoder_ppr_140:     int = 10000  # placeholder

    # ── Computed field (do not set manually) ─────────────────────────────────
    # Populated by __post_init__ from gear_ratios × encoder_ppr / 360.
    joint_counts_per_degree: Tuple[float, ...] = ()

    def __post_init__(self) -> None:
        # Joint-to-profile mapping mirrors _build_joint_drive_specs default.
        # J0..J2 → 400/750 profile, J3..J5 → 140 profile.
        ppr_by_joint = (
            self.encoder_ppr_400_750,
            self.encoder_ppr_400_750,
            self.encoder_ppr_400_750,
            self.encoder_ppr_140,
            self.encoder_ppr_140,
            self.encoder_ppr_140,
        )
        self.joint_counts_per_degree = tuple(
            (ppr_by_joint[i] * self.joint_gear_ratios[i]) / 360.0
            for i in range(6)
        )


class MotorCommanderThread(QThread):
    """Owns low-level send/read operations against RobotHardwareInterface.

    The thread keeps the latest target command and feedback snapshot. Callers
    interact through thread-safe methods to submit targets and fetch state.
    """

    def __init__(
        self,
        hardware_interface: RobotHardwareInterface,
        config: Optional[CommanderConfig] = None,
    ):
        super().__init__()
        self.hardware_interface = hardware_interface
        self.config = config or CommanderConfig()

        if len(self.config.joint_slave_ids) != 6:
            raise ValueError("CommanderConfig.joint_slave_ids must contain 6 entries")

        self._joint_drive_specs = self._build_joint_drive_specs()

        self._running = True
        self._lock = threading.Lock()

        self._pending_joints: Optional[List[float]] = None
        self._pending_joint_vel_deg_s: Optional[List[float]] = None
        self._operating_mode = OperatingMode.POSITION
        self._requested_mode = OperatingMode.POSITION
        self._latest_feedback = {
            "joints": [0.0] * 6,
            "pose": [0.0] * 6,
            "gripper": "Unlocked",
            "tool": "Unlocked",
            "timestamp": time.time(),
            "commander_mode": self._operating_mode.value,
            "dry_run_transport": bool(self.config.dry_run_transport),
        }
        self._last_command_packets: List[Dict[str, Any]] = []

        self._is_connected = False
        self._last_error = ""
        self._last_send_ok: Optional[bool] = None

    def run(self) -> None:
        period_s = 1.0 / max(1.0, float(self.config.loop_hz))
        next_tick = time.perf_counter()

        while self._running:
            try:
                if not self._is_connected:
                    self._is_connected = bool(self.hardware_interface.connect())
                    if not self._is_connected:
                        self._last_error = "connect failed"
                        time.sleep(float(self.config.reconnect_backoff_s))
                        next_tick = time.perf_counter()
                        continue

                pending = None
                pending_vel = None
                with self._lock:
                    # Phase-2 mode switch hook.
                    self._operating_mode = self._requested_mode
                    if self._pending_joints is not None:
                        pending = self._pending_joints
                        self._pending_joints = None
                    if self._pending_joint_vel_deg_s is not None:
                        pending_vel = self._pending_joint_vel_deg_s
                        self._pending_joint_vel_deg_s = None

                if self._operating_mode == OperatingMode.POSITION and pending is not None:
                    ok = self._dispatch_position_targets(pending)
                    self._last_send_ok = ok
                    if not ok:
                        self._last_error = "position dispatch failed"
                elif self._operating_mode == OperatingMode.VELOCITY and pending_vel is not None:
                    self._last_send_ok = self._dispatch_velocity_targets(pending_vel)

                feedback = self.hardware_interface.read_feedback()
                if isinstance(feedback, dict):
                    if "timestamp" not in feedback:
                        feedback = dict(feedback)
                        feedback["timestamp"] = time.time()
                    feedback = dict(feedback)
                    feedback["commander_mode"] = self._operating_mode.value
                    feedback["dry_run_transport"] = bool(self.config.dry_run_transport)
                    feedback["last_command_packets"] = list(self._last_command_packets)
                    with self._lock:
                        self._latest_feedback = feedback

            except Exception as exc:
                self._last_error = str(exc)
                self._is_connected = False
                try:
                    self.hardware_interface.disconnect()
                except Exception:
                    pass
                time.sleep(float(self.config.reconnect_backoff_s))

            next_tick += period_s
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

        try:
            self.hardware_interface.disconnect()
        except Exception:
            pass
        self._is_connected = False

    def submit_joint_targets(self, targets: List[float]) -> bool:
        """Accept the latest joint targets for dispatch.

        Returns True if accepted into the thread-safe pending slot.
        """
        if not targets or len(targets) < 6:
            return False
        with self._lock:
            self._pending_joints = [float(v) for v in targets[:6]]
        return True

    def submit_joint_velocity_targets(self, vel_deg_s: Sequence[float]) -> bool:
        """Accept the latest per-joint velocity targets in deg/s."""
        if not vel_deg_s or len(vel_deg_s) < 6:
            return False
        with self._lock:
            self._pending_joint_vel_deg_s = [float(v) for v in vel_deg_s[:6]]
        return True

    def set_operating_mode(self, mode: OperatingMode | str) -> bool:
        """Request operating mode transition for commander loop."""
        try:
            target = mode if isinstance(mode, OperatingMode) else OperatingMode(str(mode))
        except Exception:
            return False
        with self._lock:
            self._requested_mode = target
        return True

    def get_operating_mode(self) -> str:
        with self._lock:
            return self._operating_mode.value

    def get_latest_feedback(self) -> dict:
        with self._lock:
            return dict(self._latest_feedback)

    def get_connection_state(self) -> Tuple[bool, str]:
        return bool(self._is_connected), str(self._last_error)

    def consume_last_send_result(self) -> Optional[bool]:
        """Return last send result and clear it.

        None means no send result has been produced since the previous consume.
        """
        with self._lock:
            value = self._last_send_ok
            self._last_send_ok = None
            return value

    def stop_gracefully(self) -> None:
        self._running = False

    def get_joint_slave_id(self, joint_index: int) -> int:
        """Return configured Modbus slave ID for joint index 0..5."""
        idx = int(joint_index)
        if idx < 0 or idx >= 6:
            raise IndexError("joint_index must be in range 0..5")
        return int(self.config.joint_slave_ids[idx])

    def get_joint_drive_specs(self) -> Tuple[JointDriveSpec, ...]:
        """Return per-joint drive metadata used by profile adapters."""
        return tuple(self._joint_drive_specs)

    def set_dry_run_transport(self, enabled: bool) -> None:
        """Enable/disable protocol dry-run mode at runtime."""
        with self._lock:
            self.config.dry_run_transport = bool(enabled)

    def _dispatch_position_targets(self, targets_deg: Sequence[float]) -> bool:
        """Build and dispatch per-joint position packets.

        Phase-1 behavior:
        - Always build profile-aware command packets.
        - In dry-run mode, store packets for inspection and skip hardware write.
        - When dry-run is disabled, fall back to existing interface write for
          compatibility until protocol-specific transport writers are integrated.
        """
        packets = self._build_position_packets(targets_deg)
        self._last_command_packets = packets

        if self.config.dry_run_transport:
            return True

        # Compatibility path: retain legacy transport behavior until dedicated
        # profile writers (pymodbus/minimalmodbus adapters) are wired.
        return bool(self.hardware_interface.send_joint_angles(list(targets_deg)[:6]))

    def _dispatch_velocity_targets(self, vel_deg_s: Sequence[float]) -> bool:
        """Build and dispatch per-joint velocity packets.

        Velocity writes are staged in Phase-1 and run in dry-run-safe mode.
        """
        packets: List[Dict[str, Any]] = []
        for spec in self._joint_drive_specs:
            vel = float(vel_deg_s[spec.joint_index])
            packets.append(
                {
                    "joint": f"J{spec.joint_index + 1}",
                    "slave_id": int(spec.slave_id),
                    "bus": str(spec.bus_name),
                    "profile": str(spec.profile.value),
                    "mode": OperatingMode.VELOCITY.value,
                    "velocity_deg_s": float(vel),
                }
            )

        self._last_command_packets = packets
        with self._lock:
            self._latest_feedback["commanded_velocity_deg_s"] = [float(v) for v in vel_deg_s[:6]]

        if self.config.dry_run_transport:
            return True

        # Live profile-specific velocity transport to be added next phase.
        return True

    def _build_position_packets(self, targets_deg: Sequence[float]) -> List[Dict[str, Any]]:
        packets: List[Dict[str, Any]] = []
        for spec in self._joint_drive_specs:
            idx = int(spec.joint_index)
            target_deg = float(targets_deg[idx])
            scale = float(self.config.joint_counts_per_degree[idx])
            native_target = int(round(target_deg * scale))

            packet = {
                "joint": f"J{idx + 1}",
                "slave_id": int(spec.slave_id),
                "bus": str(spec.bus_name),
                "profile": str(spec.profile.value),
                "mode": OperatingMode.POSITION.value,
                "target_deg": float(target_deg),
                "target_native": int(native_target),
            }

            if spec.profile == DriveProfile.P400_750:
                packet["register_profile"] = self._position_registers_400_750()
            else:
                packet["register_profile"] = self._position_registers_140()

            packets.append(packet)
        return packets

    @staticmethod
    def _position_registers_400_750() -> Dict[str, int]:
        """Reference register map from 400w motor runner.py."""
        return {
            "REG_MODE_SELECT": 0x2109,
            "REG_OP_MODE": 0x2310,
            "REG_POS_MODE": 0x2311,
            "REG_TARGET_POS": 0x2320,
            "REG_TARGET_SPD": 0x2321,
            "REG_ACC": 0x2322,
            "REG_DEC": 0x2323,
            "REG_TRIGGER": 0x2316,
            "REG_ENABLE": 0x2301,
        }

    @staticmethod
    def _position_registers_140() -> Dict[str, int]:
        """Reference position-relevant registers from 140w motor_tester.py."""
        return {
            "REG_MODE": 24672,
            "REG_CONTROLWORD": 24640,
            "REG_ACCEL_U32": 24707,
            "REG_DECEL_U32": 24708,
            "REG_SPEED_TARGET_I32": 24831,
        }

    def _build_joint_drive_specs(self) -> Tuple[JointDriveSpec, ...]:
        if self.config.joint_drive_specs is not None:
            if len(self.config.joint_drive_specs) != 6:
                raise ValueError("CommanderConfig.joint_drive_specs must contain 6 entries")
            return tuple(self.config.joint_drive_specs)

        # Default machine inventory from user-provided topology:
        # J1/J2: 400W, J3: 750W  → bus_a (pymodbus, one USB-RS485 adapter)
        # J4/J5/J6: 140W         → bus_b (minimalmodbus, second USB-RS485 adapter)
        default_profiles = (
            (DriveProfile.P400_750, 400,  "bus_a"),
            (DriveProfile.P400_750, 400,  "bus_a"),
            (DriveProfile.P400_750, 750,  "bus_a"),
            (DriveProfile.P140,     140,  "bus_b"),
            (DriveProfile.P140,     140,  "bus_b"),
            (DriveProfile.P140,     140,  "bus_b"),
        )
        specs: List[JointDriveSpec] = []
        for idx in range(6):
            profile, power_w, bus = default_profiles[idx]
            specs.append(
                JointDriveSpec(
                    joint_index=idx,
                    slave_id=int(self.config.joint_slave_ids[idx]),
                    profile=profile,
                    power_w=int(power_w),
                    bus_name=bus,
                )
            )
        return tuple(specs)
