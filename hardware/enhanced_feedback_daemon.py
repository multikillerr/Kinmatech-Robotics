import sys
import os
import socket
import threading
import time
import json
import numpy as np
from collections import deque

# Ensure project root is in path so sibling layer modules can be found
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from planning.kin_serial import kin_engine, run_f_kin
from feedback_daemon import FeedbackDaemon

# Add parent directory to path to find rs485_motor_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import RS485 controller with fallback
try:
    from rs485_motor_controller import RS485MotorController
    RS485_AVAILABLE = True
    print("RS485 motor controller imported successfully")
except ImportError as e:
    RS485_AVAILABLE = False
    print(f"RS485 motor controller not available: {e}")
    print("Running in simulation mode - this is normal for testing")

class MockRS485Controller:
    """Mock RS485 controller for when hardware is not available"""
    def __init__(self, port=None, log_callback=None):
        self.log_callback = log_callback
        self.connected = False
        self.motor_positions = [0.0] * 6
        self.motor_ids = {i: i for i in range(1, 7)}
        
    def log(self, message):
        if self.log_callback:
            self.log_callback(f"[MOCK_RS485] {message}")
    
    def connect(self):
        self.connected = True
        self.log("Mock RS485 connection established")
        return True
    
    def disconnect(self):
        self.connected = False
        self.log("Mock RS485 disconnected")
    
    def is_connected(self):
        return self.connected
    
    def move_all_motors(self, joint_angles, speeds=None):
        self.motor_positions = joint_angles.copy()
        self.log(f"Mock motors moved to: {[f'{a:.1f}°' for a in joint_angles]}")
        return True
    
    def emergency_stop_all(self):
        self.log("Mock emergency stop activated")
        return True
    
    def get_motor_positions(self):
        return self.motor_positions.copy()
    
    def get_motor_statuses(self):
        return ['IDLE'] * 6
    
    def get_connection_status(self):
        return {
            'connected': self.connected,
            'identified_motors': 6,
            'motor_mapping': self.motor_ids,
            'feedback_frequency': 10.0,
            'last_feedback': 0.0
        }

class EnhancedFeedbackDaemon(FeedbackDaemon):
    """Enhanced feedback daemon with RS485 motor control"""
    
    def __init__(self, tester_host='localhost', rs485_port='/dev/ttyUSB0', log_callback=None):
        """
        Enhanced feedback daemon with RS485 motor control
        
        Args:
            tester_host (str): IP address of the tester machine (for GUI communication)
            rs485_port (str): RS485 interface port for motor control
            log_callback (function): Optional callback function for logging messages
        """
        # Initialize parent class
        super().__init__(tester_host, log_callback)
        
        self.rs485_port = rs485_port
        
        # Initialize motor controller
        if RS485_AVAILABLE:
            try:
                self.motor_controller = RS485MotorController(
                    port=rs485_port,
                    log_callback=self.log
                )
                self.log("Real RS485 motor controller initialized")
            except Exception as e:
                self.log(f"Failed to initialize real RS485 controller: {e}")
                self.motor_controller = MockRS485Controller(
                    port=rs485_port,
                    log_callback=self.log
                )
        else:
            self.motor_controller = MockRS485Controller(
                port=rs485_port,
                log_callback=self.log
            )
            
        self.motors_connected = False
        
        # Enhanced state tracking
        self.emergency_stop_active = False
        self.paused = False
        
        # Start motor connection in separate thread
        threading.Thread(target=self.connect_to_motors, daemon=True).start()
    
    def connect_to_motors(self):
        """Connect to RS485 motor controllers"""
        while True:
            if not self.motors_connected:
                try:
                    if self.motor_controller.connect():
                        self.motors_connected = True
                        self.log("[MOTORS] Connected to RS485 motor controllers")
                        
                        # Get initial motor positions
                        self.update_from_motors()
                    else:
                        self.log("[MOTORS] Failed to connect to motor controllers")
                        time.sleep(5)  # Retry in 5 seconds
                except Exception as e:
                    self.log(f"[MOTORS] Connection error: {e}")
                    time.sleep(5)
            else:
                # Monitor motor connection health
                if not self.motor_controller.is_connected():
                    self.log("[MOTORS] Motor connection lost")
                    self.motors_connected = False
                time.sleep(1)
    
    def process_target(self, command_type, *args):
        """Enhanced process_target with motor control integration"""
        if command_type == "HALT":
            self.emergency_stop()
            return True
        elif command_type == "PAUSE":
            self.pause_motion()
            return True
        elif command_type == "RESUME":
            self.resume_motion()
            return True
        elif command_type == "cartesian":
            return self._process_cartesian_command(*args)
        elif command_type == "joint_angles":
            return self._process_joint_command(*args)
        
        # Fallback to parent implementation
        return super().process_target(command_type, *args)
    
    def _process_cartesian_command(self, x, y, z, roll, pitch, yaw, speed, gripper_state, tool_state):
        """Process cartesian movement command"""
        if self.emergency_stop_active:
            self.log("[CMD] Move command ignored - emergency stop active")
            return False
        
        if self.paused:
            self.log("[CMD] Move command ignored - system paused")
            return False
        
        try:
            # Calculate joint angles using inverse kinematics
            joint_angles = kin_engine(x, y, z, roll, pitch, yaw)
            if joint_angles is None:
                self.log(f"[IK Error] Target unreachable: [{x:.1f}, {y:.1f}, {z:.1f}]")
                return False
            
            # Convert to degrees for motor controllers
            joint_angles_deg = [np.degrees(angle) if abs(angle) < 10 else angle for angle in joint_angles]
            
            # Send to motor controllers
            if self.motors_connected:
                motor_speeds = [speed] * 6  # Same speed for all motors
                success = self.motor_controller.move_all_motors(joint_angles_deg, motor_speeds)
                if success:
                    self.log(f"[MOTORS] Moving to: {[f'{a:.1f}°' for a in joint_angles_deg]}")
                    return True
                else:
                    self.log("[MOTORS] Failed to send move command")
                    return False
            else:
                # Fallback to network communication
                return super().process_target("cartesian", x, y, z, roll, pitch, yaw, speed, gripper_state, tool_state)
                
        except Exception as e:
            self.log(f"[CARTESIAN Error] {e}")
            return False
    
    def _process_joint_command(self, *args):
        """Process joint angle movement command"""
        if self.emergency_stop_active:
            self.log("[CMD] Move command ignored - emergency stop active")
            return False
        
        if self.paused:
            self.log("[CMD] Move command ignored - system paused")
            return False
        
        joint_angles = args[:6]
        speed = args[6]
        gripper_state = args[7]
        tool_state = args[8]
        
        try:
            # Convert to degrees if needed
            joint_angles_deg = [np.degrees(angle) if abs(angle) < 10 else angle for angle in joint_angles]
            
            # Send to motor controllers
            if self.motors_connected:
                motor_speeds = [speed] * 6
                success = self.motor_controller.move_all_motors(joint_angles_deg, motor_speeds)
                if success:
                    self.log(f"[MOTORS] Joint move: {[f'{a:.1f}°' for a in joint_angles_deg]}")
                    return True
                else:
                    self.log("[MOTORS] Failed to send joint command")
                    return False
            else:
                # Fallback to network communication
                return super().process_target("joint_angles", *args)
                
        except Exception as e:
            self.log(f"[JOINT Error] {e}")
            return False
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        self.emergency_stop_active = True
        self.log("[EMERGENCY] STOP ACTIVATED")
        
        if self.motors_connected:
            self.motor_controller.emergency_stop_all()
    
    def pause_motion(self):
        """Pause motion"""
        self.paused = True
        self.log("[CONTROL] Motion paused")
    
    def resume_motion(self):
        """Resume motion"""
        if self.emergency_stop_active:
            self.log("[CONTROL] Cannot resume - emergency stop active")
            return
        
        self.paused = False
        self.log("[CONTROL] Motion resumed")
    
    def reset_emergency_stop(self):
        """Reset emergency stop (call this manually when safe)"""
        self.emergency_stop_active = False
        self.log("[EMERGENCY] Stop reset")
    
    def update_from_motors(self):
        """Update current state from motor feedback"""
        if not self.motors_connected:
            return
        
        try:
            # Get motor positions
            motor_positions = self.motor_controller.get_motor_positions()
            if motor_positions and len(motor_positions) == 6:
                self.current_joint_angles = motor_positions.copy()
                
                # Convert to radians for forward kinematics
                joint_angles_rad = [np.radians(angle) for angle in motor_positions]
                
                # Calculate cartesian position using forward kinematics
                position = run_f_kin(*joint_angles_rad)
                if position:
                    self.current_position = list(position)
                
                # Track update frequency
                self.update_frequency.add_event()
                
        except Exception as e:
            self.log(f"[FEEDBACK] Error updating from motors: {e}")
    
    def get_current_position(self):
        """Get current position with motor feedback integration"""
        if self.motors_connected:
            self.update_from_motors()
        return super().get_current_position()
    
    def get_current_joint_angles(self):
        """Get current joint angles with motor feedback integration"""
        if self.motors_connected:
            self.update_from_motors()
        return super().get_current_joint_angles()
    
    def is_connected(self):
        """Check if both network and motors are connected"""
        network_connected = super().is_connected()
        return network_connected or self.motors_connected  # Either connection is sufficient
    
    @property
    def network_connected(self):
        """Check network connection status"""
        return super().is_connected()
    
    def disconnect(self):
        """Disconnect all systems"""
        self.log("[SHUTDOWN] Disconnecting all systems...")
        
        # Stop motors first
        if self.motors_connected:
            self.motor_controller.emergency_stop_all()
            self.motor_controller.disconnect()
        
        # Call parent disconnect
        super().disconnect()
        
        self.log("[SHUTDOWN] All systems disconnected")

# Example usage
if __name__ == "__main__":
    def log_callback(message):
        print(f"[LOG] {message}")
    
    # Create enhanced daemon
    daemon = EnhancedFeedbackDaemon(
        tester_host='localhost',
        rs485_port='/dev/ttyUSB0',  # Adjust for your system
        log_callback=log_callback
    )
    
    try:
        print("Enhanced Feedback Daemon started...")
        print("Commands:")
        print("1. Test move: daemon.process_target('cartesian', 150, 0, 300, 0, 90, 0, 50, 'Locked', 'Locked')")
        print("2. Emergency stop: daemon.emergency_stop()")
        print("3. Reset emergency: daemon.reset_emergency_stop()")
        print("4. Get status: daemon.is_connected()")
        
        # Keep running
        while True:
            time.sleep(1)
            
            # Print status every 10 seconds
            if int(time.time()) % 10 == 0:
                status = "CONNECTED" if daemon.is_connected() else "DISCONNECTED"
                freq = daemon.get_update_frequency()
                print(f"Status: {status}, Frequency: {freq:.1f} Hz")
    
    except KeyboardInterrupt:
        print("\nShutting down...")
        daemon.disconnect()
