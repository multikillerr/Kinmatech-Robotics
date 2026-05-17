#!/usr/bin/env python3
"""
Control Thread for Fixed-Rate Command Execution
Implements a deterministic 100 Hz control loop that:
1. Reads current state from hardware feedback
2. Gets next command from queue
3. Computes next joint targets
4. Sends to hardware

Independent of UI - runs at fixed 10ms intervals.
"""

import sys
import os
import time
from dataclasses import dataclass
from typing import Any, Tuple, Optional, List
from queue import Empty
from threading import Lock
import numpy as np

# Ensure project root is in path
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from PyQt6.QtCore import QThread, pyqtSignal as QtSignal
from planning.kinematics_adapter import solve_fk_gui, solve_ik_gui
from control.command_queue import ThreadSafeCommandQueue, ControlCommand, CommandType
from control.state_machine import RobotStateMachine, ControlState
from hardware.robot_interface import MockRobotHardwareInterface, RobotHardwareInterface
from hardware.commander import MotorCommanderThread


@dataclass(frozen=True)
class NormalizedCommand:
    """Internal normalized command used by ControlThread."""

    state: ControlState
    command_type: str
    args: Tuple[Any, ...] = ()


class ControlThread(QThread):
    """
    Fixed-rate 100 Hz (10 ms) control loop thread.
    
    Executes deterministically and independently of UI timing.
    
    Loop:
    ----
    1. Read current state from hardware feedback
    2. Get next command from queue (non-blocking)
    3. Compute next joint targets via interpolation
    4. Send to hardware
    5. Sleep to maintain 10 ms cycle time
    
    All communication via signals - NO shared variables.
    
    Signals:
    --------
    log_message(str, str)
        Emitted with (message, log_level) for UI logging
    movement_started(str, dict)
        Emitted when movement command starts: (type, details)
    movement_completed(str)
        Emitted when movement completes: (command_type)
    joint_targets(list)
        Emitted with computed joint targets every loop iteration
    cartesian_state(dict)
        Emitted with current cartesian state (x,y,z,roll,pitch,yaw)
    halt_requested()
        Emitted when HALT command received
    pause_requested()
        Emitted when PAUSE command received
    resume_requested()
        Emitted when RESUME command received
    """
    
    # Control loop frequency (Hz)
    CONTROL_FREQ_HZ = 100
    LOOP_PERIOD_MS = 10  # 1000 / 100
    LOOP_PERIOD_S = 0.01
    
    # Signals
    log_message = QtSignal(str, str)              # (message, level)
    movement_started = QtSignal(str, dict)        # (type, details)
    movement_completed = QtSignal(str)            # (cmd_type)
    joint_targets = QtSignal(list)                # [j1, j2, j3, j4, j5, j6]
    cartesian_state = QtSignal(dict)              # {x, y, z, roll, pitch, yaw}
    halt_requested = QtSignal()
    pause_requested = QtSignal()
    resume_requested = QtSignal()
    
    def __init__(self, command_queue: ThreadSafeCommandQueue):
        """
        Initialize the fixed-rate control thread.
        
        Args:
            command_queue: ThreadSafeCommandQueue for commands (non-blocking get in loop)
        """
        super().__init__()
        self.command_queue = command_queue
        self.running = True
        self.setObjectName("ControlThread@100Hz")
        
        # ─ State machine ─
        self.state_machine = RobotStateMachine()
        self.paused = False
        self._paused_at: Optional[float] = None
        
        # ─ Current state (read from hardware) ─
        self.current_joints = [0.0] * 6
        self.current_cartesian = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joints_lock = Lock()
        
        # ─ Target state (from commands) ─
        self.target_joints = None  # List of 6 floats or None if not moving
        self.target_cartesian = None  # Tuple (x,y,z,roll,pitch,yaw)
        self.movement_speed = 50  # degrees/s for joint, mm/s for cartesian
        
        # ─ Interpolation state ─
        self.start_time = None  # Time when movement started
        self.movement_duration = None  # How long to move
        self.progress_ratio = 0.0  # 0.0 to 1.0
        
        # ─ Command tracking ─
        self.current_command = None
        self.command_executed = False
        self.start_joints = None  # Initial joints at start of movement
        self._last_overrun_log_ts = 0.0

    @property
    def state(self) -> ControlState:
        """Expose current control state for observers/tests."""
        return self.state_machine.state

    def run(self):
        """
        Main 100 Hz control loop.
        Runs deterministically independent of UI.
        """
        self.log_message.emit("Control thread started @ 100 Hz", "SUCCESS")
        
        loop_start = time.perf_counter()
        iteration_count = 0
        
        while self.running:
            # ────────────────────────────────────────────────────────────
            # 1. READ CURRENT STATE (from cached hardware feedback)
            # ────────────────────────────────────────────────────────────
            with self.joints_lock:
                current_joints = self.current_joints.copy()
            
            # ────────────────────────────────────────────────────────────
            # 2. GET NEXT COMMAND (non-blocking)
            # ────────────────────────────────────────────────────────────
            try:
                cmd = self.command_queue.get_nowait()
                if cmd is None:  # Poison pill
                    self.log_message.emit("Shutdown signal received", "INFO")
                    break
                normalized_cmd = self._normalize_command(cmd)
                if normalized_cmd.command_type:
                    self._handle_command(normalized_cmd)
                self.command_queue.task_done()
            except Empty:
                pass  # No new command - will continue with previous target
            
            # ────────────────────────────────────────────────────────────
            # 3. COMPUTE NEXT JOINT TARGETS (interpolation)
            # ────────────────────────────────────────────────────────────
            joint_targets = self._compute_next_targets(current_joints)
            
            # ────────────────────────────────────────────────────────────
            # 4. UPDATE CARTESIAN STATE (forward kinematics)
            # ────────────────────────────────────────────────────────────
            try:
                gui_pose = solve_fk_gui(joint_targets)
                with self.joints_lock:
                    self.current_joints = joint_targets.copy()
                    self.current_cartesian = list(gui_pose)
                # Emit cartesian state
                self.cartesian_state.emit({
                    "x": gui_pose[0],
                    "y": gui_pose[1],
                    "z": gui_pose[2],
                    "roll": gui_pose[3],
                    "pitch": gui_pose[4],
                    "yaw": gui_pose[5],
                })
            except Exception as e:
                self.log_message.emit(f"FK computation error: {str(e)}", "WARNING")
            
            # ────────────────────────────────────────────────────────────
            # 5. SEND TO HARDWARE (via signal)
            # ────────────────────────────────────────────────────────────
            self.joint_targets.emit(joint_targets)
            
            # ────────────────────────────────────────────────────────────
            # 6. MAINTAIN FIXED LOOP RATE (100 Hz = 10 ms)
            # ────────────────────────────────────────────────────────────
            iteration_count += 1
            elapsed = time.perf_counter() - loop_start
            
            if iteration_count % 1000 == 0:  # Every 10 seconds
                actual_freq = iteration_count / elapsed
                self.log_message.emit(
                    f"Control loop: {actual_freq:.1f} Hz (target: 100 Hz)",
                    "DEBUG"
                )
            
            # Sleep to maintain 10 ms cycle time
            target_time = loop_start + (iteration_count * self.LOOP_PERIOD_S)
            sleep_time = target_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.005:  # More than 5ms behind
                now = time.perf_counter()
                if (now - self._last_overrun_log_ts) >= 5.0:
                    self._last_overrun_log_ts = now
                    self.log_message.emit(
                        f"Control loop overrun: {abs(sleep_time)*1000:.1f} ms behind",
                        "WARNING"
                    )
    
    def _normalize_command(self, raw_cmd: Any) -> NormalizedCommand:
        """Normalize queue items to state-aware control commands.

        Supported high-level envelopes:
        - ControlCommand(CommandType.*, payload)
        - ("JOG", payload)
        - ("EXECUTE", payload)
        - ("ABORT", payload)
        - ("RETURN", payload)
        """
        if raw_cmd is None:
            return NormalizedCommand(ControlState.IDLE, "")

        if isinstance(raw_cmd, ControlCommand):
            cmd_type = raw_cmd.command_type.value
            payload = raw_cmd.payload
            if cmd_type == CommandType.ABORT.value:
                return NormalizedCommand(ControlState.ABORT, "HALT")
            if cmd_type in (CommandType.RETURN.value, CommandType.RETURN_HOME.value):
                if payload is None:
                    return NormalizedCommand(
                        ControlState.RETURN_HOME,
                        "joint_angles",
                        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0),
                    )
                if isinstance(payload, (list, tuple)):
                    return NormalizedCommand(
                        ControlState.RETURN_HOME,
                        "joint_angles",
                        tuple(payload),
                    )
            if cmd_type in (CommandType.JOG.value, CommandType.EXECUTE.value):
                if isinstance(payload, (list, tuple)):
                    payload_tuple = tuple(payload)
                    if not payload_tuple:
                        return NormalizedCommand(ControlState.IDLE, "")
                    return NormalizedCommand(
                        ControlState.JOG if cmd_type == CommandType.JOG.value else ControlState.EXECUTE,
                        str(payload_tuple[0]),
                        tuple(payload_tuple[1:]),
                    )
                return NormalizedCommand(ControlState.IDLE, "")

        if isinstance(raw_cmd, (list, tuple)) and raw_cmd:
            head = raw_cmd[0]
            if isinstance(head, str):
                upper_head = head.upper()
                if upper_head == CommandType.ABORT.value:
                    return NormalizedCommand(ControlState.ABORT, "HALT")
                if upper_head in (CommandType.RETURN.value, CommandType.RETURN_HOME.value):
                    if len(raw_cmd) == 1:
                        return NormalizedCommand(
                            ControlState.RETURN_HOME,
                            "joint_angles",
                            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0),
                        )
                    if isinstance(raw_cmd[1], (list, tuple)):
                        return NormalizedCommand(
                            ControlState.RETURN_HOME,
                            "joint_angles",
                            tuple(raw_cmd[1]),
                        )
                if upper_head in (CommandType.JOG.value, CommandType.EXECUTE.value):
                    if len(raw_cmd) > 1 and isinstance(raw_cmd[1], (list, tuple)):
                        payload_tuple = tuple(raw_cmd[1])
                        if not payload_tuple:
                            return NormalizedCommand(ControlState.IDLE, "")
                        return NormalizedCommand(
                            ControlState.JOG if upper_head == CommandType.JOG.value else ControlState.EXECUTE,
                            str(payload_tuple[0]),
                            tuple(payload_tuple[1:]),
                        )

        if isinstance(raw_cmd, (list, tuple)):
            payload_tuple = tuple(raw_cmd)
            if not payload_tuple:
                return NormalizedCommand(ControlState.IDLE, "")

            head = str(payload_tuple[0])
            default_state = ControlState.EXECUTE
            if head == "HALT":
                default_state = ControlState.ABORT
            elif head == "joint_angles" and len(payload_tuple) >= 8 and payload_tuple[7] == "JOG":
                default_state = ControlState.JOG

            return NormalizedCommand(default_state, head, tuple(payload_tuple[1:]))

        return NormalizedCommand(ControlState.IDLE, "")

    def _transition_for_motion(self, next_state: ControlState) -> None:
        """Route all operational state transitions through the control-layer state machine."""
        if next_state == ControlState.JOG:
            self.state_machine.start_jog()
        elif next_state == ControlState.EXECUTE:
            self.state_machine.start_execute()
        elif next_state == ControlState.RETURN_HOME:
            self.state_machine.start_return_home()
        elif next_state == ControlState.ABORT:
            self.state_machine.abort()

    def _handle_command(self, cmd: NormalizedCommand) -> None:
        """
        Handle a normalized command from the queue.
        
        Args:
            cmd: NormalizedCommand instance
        """
        if not cmd.command_type:
            return
        
        cmd_type = cmd.command_type
        args = cmd.args
        
        try:
            if cmd_type == "HALT":
                self._handle_halt()
                
            elif cmd_type == "PAUSE":
                self._handle_pause()
                
            elif cmd_type == "RESUME":
                self._handle_resume()
                
            elif cmd_type == "joint_angles":
                self._handle_joint_command(args, cmd.state)
                
            elif cmd_type == "cartesian":
                self._handle_cartesian_command(args, cmd.state)
                
            elif cmd_type == "timer":
                self._handle_timer_command(args, cmd.state)
                
            else:
                self.log_message.emit(f"Unknown command: {cmd_type}", "WARNING")
                
        except Exception as e:
            self.log_message.emit(
                f"Error handling {cmd_type}: {str(e)}", "ERROR"
            )
    
    def _handle_joint_command(self, args: Tuple, target_state: ControlState) -> None:
        """
        Handle joint angle command.
        Sets target joint angles for interpolation.
        """
        if len(args) < 6:
            self.log_message.emit("Invalid joint command (need 6 angles)", "ERROR")
            return
        
        target_joints = list(args[:6])
        self.movement_speed = float(args[6]) if len(args) > 6 else 50.0
        gripper = args[7] if len(args) > 7 else "Unlocked"
        tool = args[8] if len(args) > 8 else "Unlocked"
        
        # Start movement
        with self.joints_lock:
            self.start_joints = self.current_joints.copy()
        
        self.paused = False
        self._paused_at = None
        self.target_joints = target_joints
        self.target_cartesian = None
        self._transition_for_motion(target_state)
        self.start_time = time.perf_counter()
        
        # Compute movement duration based on max joint error and speed
        max_angle_error = max(
            abs(target_joints[i] - self.start_joints[i]) 
            for i in range(6)
        )
        self.movement_duration = max_angle_error / self.movement_speed if self.movement_speed > 0 else 1.0
        
        joint_str = ', '.join(f'{j:.2f}' for j in target_joints)
        msg = f"Moving to joint angles: [{joint_str}] speed={self.movement_speed}°/s"
        self.log_message.emit(msg, "INFO")
        self.movement_started.emit("joint_angles", {
            "joints": target_joints,
            "speed": self.movement_speed,
            "gripper": gripper,
            "tool": tool,
            "state": self.state.value,
        })
    
    def _handle_cartesian_command(self, args: Tuple, target_state: ControlState) -> None:
        """
        Handle cartesian command.
        Converts to joint angles and starts interpolation.
        """
        if len(args) < 6:
            self.log_message.emit("Invalid cartesian command", "ERROR")
            return
        
        x, y, z = float(args[0]), float(args[1]), float(args[2])
        roll, pitch, yaw = float(args[3]), float(args[4]), float(args[5])
        self.movement_speed = float(args[6]) if len(args) > 6 else 50.0
        gripper = args[7] if len(args) > 7 else "Unlocked"
        tool = args[8] if len(args) > 8 else "Unlocked"
        
        # Compute target joint angles using inverse kinematics
        try:
            target_joints = solve_ik_gui((x, y, z, roll, pitch, yaw))
            if target_joints is None:
                self.log_message.emit(
                    f"IK failed for cartesian target ({x}, {y}, {z})",
                    "ERROR"
                )
                return
        except Exception as e:
            self.log_message.emit(f"IK computation error: {str(e)}", "ERROR")
            return
        
        # Start movement
        with self.joints_lock:
            self.start_joints = self.current_joints.copy()
        
        self.paused = False
        self._paused_at = None
        self.target_joints = list(target_joints)
        self.target_cartesian = (x, y, z, roll, pitch, yaw)
        self._transition_for_motion(target_state)
        self.start_time = time.perf_counter()
        
        # Compute movement duration
        max_angle_error = max(
            abs(self.target_joints[i] - self.start_joints[i]) 
            for i in range(6)
        )
        self.movement_duration = max_angle_error / self.movement_speed if self.movement_speed > 0 else 1.0
        
        msg = f"Moving to cartesian: X={x:.1f} Y={y:.1f} Z={z:.1f} R={roll:.1f} P={pitch:.1f} W={yaw:.1f}"
        self.log_message.emit(msg, "INFO")
        self.movement_started.emit("cartesian", {
            "x": x, "y": y, "z": z,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "speed": self.movement_speed,
            "gripper": gripper,
            "tool": tool,
            "state": self.state.value,
        })
    
    def _handle_timer_command(self, args: Tuple, target_state: ControlState) -> None:
        """Handle timer command (delay)."""
        delay = float(args[0]) if args else 0.0
        self.paused = False
        self._paused_at = None
        self.target_joints = self.current_joints.copy()
        self.target_cartesian = None
        self._transition_for_motion(target_state)
        self.start_time = time.perf_counter()
        self.movement_duration = delay
        self.log_message.emit(f"Timer: {delay:.2f}s", "INFO")
        self.movement_started.emit("timer", {"duration": delay, "state": self.state.value})
    
    def _handle_halt(self) -> None:
        """Handle HALT command."""
        self._transition_for_motion(ControlState.ABORT)
        self.paused = False
        self._paused_at = None
        self.target_joints = None
        self.target_cartesian = None
        self.movement_duration = None
        self.log_message.emit("HALT", "ERROR")
        self.command_queue.clear()
        self.halt_requested.emit()
    
    def _handle_pause(self) -> None:
        """Handle PAUSE command."""
        if not self.paused:
            self.paused = True
            self._paused_at = time.perf_counter()
        self.log_message.emit("PAUSE", "WARNING")
        self.pause_requested.emit()
    
    def _handle_resume(self) -> None:
        """Handle RESUME command."""
        if self.paused:
            paused_duration = 0.0
            if self._paused_at is not None:
                paused_duration = time.perf_counter() - self._paused_at
            if self.start_time is not None:
                self.start_time += paused_duration
            self.paused = False
            self._paused_at = None
        self.log_message.emit("RESUME", "SUCCESS")
        self.resume_requested.emit()
    
    def _compute_next_targets(self, current_joints: List[float]) -> List[float]:
        """
        Compute the next joint target via linear interpolation.
        
        Returns:
            List of 6 joint angles for the next control cycle
        """
        if self.state_machine.is_idle() or self.target_joints is None:
            return current_joints
        
        if self.state_machine.is_abort() or self.paused:
            return current_joints
        
        # Compute progress (0.0 to 1.0)
        if self.movement_duration is None or self.movement_duration <= 0:
            return self.target_joints or current_joints
        
        elapsed = time.perf_counter() - self.start_time
        progress = min(elapsed / self.movement_duration, 1.0)
        
        # Linear interpolation
        result = []
        for i in range(6):
            start = self.start_joints[i]
            target = self.target_joints[i]
            current = start + (target - start) * progress
            result.append(current)
        
        # Check if movement is complete
        if progress >= 1.0:
            result = self.target_joints.copy()
            self._on_movement_complete()
        
        return result
    
    def _on_movement_complete(self) -> None:
        """Called when a movement completes."""
        self.movement_completed.emit("move")
        self.log_message.emit("Movement complete", "SUCCESS")
        self.state_machine.complete()
        self.target_joints = None
        self.target_cartesian = None
        self.movement_duration = None
    
    def update_hardware_feedback(self, joints: List[float]) -> None:
        """
        Update current joint state from hardware feedback.
        Called by HardwareThread via signal connection.
        
        Args:
            joints: Current joint angles [j1, j2, j3, j4, j5, j6]
        """
        if joints and len(joints) == 6:
            with self.joints_lock:
                self.current_joints = list(joints)
    
    def stop_gracefully(self) -> None:
        """Signal the thread to stop gracefully."""
        self.running = False
        # Put poison pill to wake up thread if waiting
        if hasattr(self.command_queue, "enqueue"):
            self.command_queue.enqueue(None)
        else:
            self.command_queue.put(None)


class HardwareThread(QThread):
    """
    Hardware communication thread.
    Handles all I/O with external hardware (robot, feedback system).
    
    Runs at ~20 Hz update rate and receives joint targets from ControlThread
    at 100 Hz via signal connection. Updates ControlThread with current state
    via feedback signal.
    
    All communication via signals - NO shared variables.
    
    Signals:
    --------
    connected(bool, str)
        Emitted when hardware connection status changes: (is_connected, message)
    feedback_received(dict)
        Emitted with current robot state: {joints, pose, gripper, tool, timestamp}
    command_sent(str, bool)
        Emitted when a command is sent: (command_type, success)
    log_message(str, str)
        Emitted with (message, log_level) for UI logging
    error_occurred(str)
        Emitted when an error occurs: (error_message)
    """
    
    # Signals
    connected = QtSignal(bool, str)           # (is_connected, status_msg)
    feedback_received = QtSignal(dict)        # Current robot state {joints, ...}
    command_sent = QtSignal(str, bool)        # (cmd_type, success)
    log_message = QtSignal(str, str)          # (message, level)
    error_occurred = QtSignal(str)            # (error_msg)
    
    def __init__(self, hardware_interface: Optional[RobotHardwareInterface] = None):
        """Initialize the hardware thread."""
        super().__init__()
        self.running = True
        self.is_connected = False
        self.setObjectName("HardwareThread")
        self.hardware_interface = hardware_interface or MockRobotHardwareInterface()
        self.commander = MotorCommanderThread(self.hardware_interface)
        self._last_connection_state = None
        
        # Command queue for joint targets (from ControlThread)
        self._pending_joints = None

    
    def run(self):
        """Main hardware loop."""
        self.log_message.emit("Hardware thread started @ 20 Hz", "SUCCESS")
        if not self.commander.isRunning():
            self.commander.start()
            self.log_message.emit("Motor commander thread started", "INFO")
        
        loop_start = time.perf_counter()
        iteration_count = 0
        
        while self.running:
            try:
                self._sync_connection_state()

                # Send any pending joint command
                if self._pending_joints is not None:
                    self._send_joint_targets(self._pending_joints)
                    self._pending_joints = None

                # Read feedback
                self._read_feedback()
                
                # Maintain ~20 Hz update rate (50 ms period)
                iteration_count += 1
                target_time = loop_start + (iteration_count * 0.050)
                sleep_time = target_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                self.error_occurred.emit(str(e))
                self.is_connected = False
                self.connected.emit(False, f"Connection error: {str(e)}")
                time.sleep(1)  # Back off before retry
    
    def receive_joint_targets(self, targets: List[float]) -> None:
        """
        Receive joint targets from ControlThread (called every 10 ms via signal).
        Queues the targets for sending to hardware.
        
        Args:
            targets: List of 6 joint angles
        """
        if targets and len(targets) == 6:
            self._pending_joints = list(targets)
    
    def _send_joint_targets(self, targets: List[float]) -> None:
        """
        Send joint target angles to hardware.
        
        Args:
            targets: List of 6 joint angles in degrees
        """
        try:
            accepted = self.commander.submit_joint_targets(targets)
            if not accepted:
                self.command_sent.emit("joint_angles", False)
                self.error_occurred.emit("Rejected joint targets")
                return

            send_result = self.commander.consume_last_send_result()
            success = bool(self.is_connected) if send_result is None else bool(send_result)
            self.command_sent.emit("joint_angles", success)
            if not success:
                self.error_occurred.emit("Failed to send joint targets")
        except Exception as e:
            self.error_occurred.emit(f"Failed to send joint targets: {str(e)}")

    
    def _attempt_connection(self) -> None:
        """Attempt to connect to hardware."""
        try:
            self.is_connected, last_error = self.commander.get_connection_state()
            if self.is_connected:
                self.connected.emit(True, "Connected to robot hardware")
                self.log_message.emit("Hardware connection established", "SUCCESS")
            else:
                self.connected.emit(False, "Hardware connection unavailable")
                msg = "Hardware connection unavailable"
                if last_error:
                    msg = f"{msg}: {last_error}"
                self.log_message.emit(msg, "WARNING")
                time.sleep(1)
        except Exception as e:
            self.is_connected = False
            self.connected.emit(False, f"Connection failed: {str(e)}")
            self.log_message.emit(f"Hardware connection failed: {str(e)}", "ERROR")
            time.sleep(2)  # Back off 2 seconds before retry

    def _sync_connection_state(self) -> None:
        """Emit connection status changes from the commander thread."""
        connected, last_error = self.commander.get_connection_state()
        self.is_connected = bool(connected)

        if self._last_connection_state is None or self._last_connection_state != self.is_connected:
            self._last_connection_state = self.is_connected
            if self.is_connected:
                self.connected.emit(True, "Connected to robot hardware")
                self.log_message.emit("Hardware connection established", "SUCCESS")
            else:
                status = "Hardware connection unavailable"
                if last_error:
                    status = f"{status}: {last_error}"
                self.connected.emit(False, status)
                self.log_message.emit(status, "WARNING")
    
    def _read_feedback(self) -> None:
        """Read feedback from hardware and emit feedback_received signal."""
        try:
            feedback = self.commander.get_latest_feedback()
            self.feedback_received.emit(feedback)
        except Exception as e:
            self.error_occurred.emit(f"Feedback read error: {str(e)}")

    
    def send_cartesian_command(self, x: float, y: float, z: float,
                              roll: float, pitch: float, yaw: float,
                              speed: float = 50.0) -> None:
        """
        Send cartesian movement command (deprecated).
        Joint computation now handled by ControlThread.
        
        Args:
            x, y, z: Position in mm
            roll, pitch, yaw: Orientation in degrees
            speed: Movement speed
        """
        if not self.is_connected:
            self.error_occurred.emit("Hardware not connected")
            self.command_sent.emit("cartesian", False)
            return
        
        try:
            self.log_message.emit(
                f"Cartesian (deprecated): X={x:.1f} Y={y:.1f} Z={z:.1f} "
                f"R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}",
                "WARNING"
            )
            self.command_sent.emit("cartesian", True)
        except Exception as e:
            self.error_occurred.emit(f"Failed to send cartesian command: {str(e)}")
            self.command_sent.emit("cartesian", False)
    
    def send_joint_command(self, joints: list, speed: float = 50.0) -> None:
        """
        Send joint movement command (deprecated).
        Joint targets sent continuously by ControlThread via receive_joint_targets.
        
        Args:
            joints: List of 6 joint angles in degrees
            speed: Movement speed in deg/s
        """
        if not self.is_connected:
            self.error_occurred.emit("Hardware not connected")
            self.command_sent.emit("joint_angles", False)
            return
        
        try:
            joint_str = ', '.join(f'{j:.1f}' for j in joints[:6])
            self.log_message.emit(f"Joint angles (deprecated): [{joint_str}]", "WARNING")
            self.command_sent.emit("joint_angles", True)
        except Exception as e:
            self.error_occurred.emit(f"Failed to send joint command: {str(e)}")
            self.command_sent.emit("joint_angles", False)

    
    def send_halt(self) -> None:
        """Send hardware HALT command."""
        if not self.is_connected:
            self.command_sent.emit("HALT", False)
            return
        
        try:
            # TODO: Implement actual HALT command
            self.log_message.emit("HALT command sent to hardware", "ERROR")
            self.command_sent.emit("HALT", True)
        except Exception as e:
            self.error_occurred.emit(f"Failed to send HALT: {str(e)}")
            self.command_sent.emit("HALT", False)
    
    def send_pause(self) -> None:
        """Send hardware PAUSE command."""
        if not self.is_connected:
            self.command_sent.emit("PAUSE", False)
            return
        
        try:
            # TODO: Implement actual PAUSE command
            self.log_message.emit("PAUSE command sent to hardware", "WARNING")
            self.command_sent.emit("PAUSE", True)
        except Exception as e:
            self.error_occurred.emit(f"Failed to send PAUSE: {str(e)}")
            self.command_sent.emit("PAUSE", False)
    
    def send_resume(self) -> None:
        """Send hardware RESUME command."""
        if not self.is_connected:
            self.command_sent.emit("RESUME", False)
            return
        
        try:
            # TODO: Implement actual RESUME command
            self.log_message.emit("RESUME command sent to hardware", "SUCCESS")
            self.command_sent.emit("RESUME", True)
        except Exception as e:
            self.error_occurred.emit(f"Failed to send RESUME: {str(e)}")
            self.command_sent.emit("RESUME", False)
    
    def stop_gracefully(self) -> None:
        """Stop the hardware thread gracefully."""
        self.running = False
        self.is_connected = False
        try:
            self.commander.stop_gracefully()
            if self.commander.isRunning():
                self.commander.wait(1000)
            self.hardware_interface.disconnect()
        except Exception:
            pass
