import sys
import os
import socket
import threading
import time
import json
from collections import deque

# Ensure project root is in path so sibling layer modules can be found
_this_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_this_dir)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from planning.kin_serial import kin_engine, run_f_kin

class FrequencyCalculator:
    def __init__(self, window_size=10):
        self.timestamps = deque(maxlen=window_size)
        self.window_size = window_size
    
    def add_event(self):
        """Record a new event timestamp"""
        self.timestamps.append(time.time())
    
    def get_frequency(self):
        """Calculate frequency in events per second"""
        if len(self.timestamps) < 2:
            return 0.0
        
        time_span = self.timestamps[-1] - self.timestamps[0]
        if time_span == 0:
            return 0.0
        
        return (len(self.timestamps) - 1) / time_span

class FeedbackDaemon:
    def __init__(self, tester_host='localhost', log_callback=None):
        """
        Initialize the feedback daemon
        
        Args:
            tester_host (str): IP address of the tester machine
            log_callback (function): Optional callback function for logging messages
        """
        self.tester_host = tester_host
        self.log_callback = log_callback
        self.cmd_sock = None
        self.fb_sock = None
        self.connected = False
        self.last_target = None
        
        # Initialize frequency calculator for updates
        self.update_frequency = FrequencyCalculator(window_size=20)
        
        # Current position storage
        self.current_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Start connection thread
        threading.Thread(target=self.connect_to_tester, daemon=True).start()
    
    def log(self, message):
        """Log a message using the callback or print"""
        if self.log_callback:
            self.log_callback(message)
        else:
            print(message)
    
    def connect_to_tester(self):
        """Connect to tester's command and feedback servers"""
        while True:
            if not self.connected:
                try:
                    # Connect to tester's command server
                    self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.cmd_sock.connect((self.tester_host, 5555))
                    
                    # Connect to tester's feedback server
                    self.fb_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.fb_sock.connect((self.tester_host, 5556))
                    
                    self.connected = True
                    self.log(f"[FEEDBACK_DAEMON] Connected to tester at {self.tester_host}")
                    
                    # Start feedback thread
                    threading.Thread(target=self.handle_feedback, daemon=True).start()
                    
                except Exception as e:
                    self.log(f"[FEEDBACK_DAEMON] Connection to {self.tester_host} failed: {e}")
                    self.connected = False # Ensure connected is false before retrying
                    time.sleep(1)
            else:
                # Periodically check the connection health
                try:
                    # Send a small, non-disruptive message to check the command socket
                    self.cmd_sock.sendall(b'\n') 
                except (socket.error, BrokenPipeError):
                    self.log("[FEEDBACK_DAEMON] Connection lost. Reconnecting...")
                    self.connected = False
                time.sleep(5) # Check every 5 seconds
    
    def process_target(self, command_type, *args):
        """Process target position through inverse kinematics or directly send joint angles"""
        if not self.connected:
            self.log("[FEEDBACK_DAEMON] Not connected to tester")
            return False

        if command_type == "HALT":
            message_data = {"command": "HALT"}
            message = json.dumps(message_data) + '\n'
            self.cmd_sock.sendall(message.encode())
            self.log("[FEEDBACK_DAEMON] Sent HALT command to tester")
            return True
        elif command_type == "PAUSE":
            message_data = {"command": "PAUSE"}
            message = json.dumps(message_data) + '\n'
            self.cmd_sock.sendall(message.encode())
            self.log("[FEEDBACK_DAEMON] Sent PAUSE command to tester")
            return True
        elif command_type == "RESUME":
            message_data = {"command": "RESUME"}
            message = json.dumps(message_data) + '\n'
            self.cmd_sock.sendall(message.encode())
            self.log("[FEEDBACK_DAEMON] Sent RESUME command to tester")
            return True
        elif command_type == "cartesian":
            x, y, z, roll, pitch, yaw, speed, gripper_state, tool_state = args
            target = [x, y, z, roll, pitch, yaw]
            # No need to check last_target here, as the GUI handles it for continuous moves
            try:
                # Step 1: Calculate joint angles using inverse kinematics
                joint_angles = kin_engine(x, y, z, roll, pitch, yaw)
                if joint_angles is None: # kin_engine can return None if unreachable
                    self.log(f"[IK Error] Target unreachable: [{x:.1f}, {y:.1f}, {z:.1f}, {roll:.1f}, {pitch:.1f}, {yaw:.1f}]")
                    return False

                # Step 2: Send joint angles, speed, gripper, tool to tester
                message_data = {
                    "command": "MOVE",
                    "joint_angles": list(joint_angles),
                    "speed": speed,
                    "gripper": gripper_state,
                    "tool": tool_state
                }
                message = json.dumps(message_data) + '\n'
                self.cmd_sock.sendall(message.encode())
                self.log(f"[FEEDBACK_DAEMON] Sent cartesian target to tester: {message_data}")
                
                self.last_target = target # Update last_target for cartesian moves
                return True
                
            except Exception as e:
                self.log(f"[IK Error] {e}")
                return False
        elif command_type == "joint_angles":
            # args will be (j1, j2, j3, j4, j5, j6, speed, gripper, tool)
            joint_angles = args[:6]
            speed = args[6]
            gripper_state = args[7]
            tool_state = args[8]
            try:
                # Directly send joint angles, speed, gripper, tool to tester
                message_data = {
                    "command": "MOVE",
                    "joint_angles": list(joint_angles),
                    "speed": speed,
                    "gripper": gripper_state,
                    "tool": tool_state
                }
                message = json.dumps(message_data) + '\n'
                self.cmd_sock.sendall(message.encode())
                self.log(f"[FEEDBACK_DAEMON] Sent pre-calculated joint angles to tester: {message_data}")
                return True
            except Exception as e:
                self.log(f"[JOINT_SEND Error] {e}")
                return False
        return False
    
    def handle_feedback(self):
        """Handle feedback from tester - Step 3: Receive joint angles and calculate actual position"""
        buffer = ""
        while self.is_connected():
            try:
                data = self.fb_sock.recv(1024).decode()
                if not data:
                    self.log("[FEEDBACK_DAEMON] Feedback stream closed by tester.")
                    self.connected = False
                    break
                    
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        feedback = json.loads(line)
                        joint_angles = feedback.get('joint_angles')
                        
                        if joint_angles and len(joint_angles) == 6:
                            # Step 4: Calculate actual position using forward kinematics
                            position = run_f_kin(*joint_angles)
                            # self.log(f"[FB] Received angles: {[round(a, 2) for a in joint_angles]}")
                            # self.log(f"[FK] Actual position: {[round(p, 2) for p in position]}")
                            
                            # Update current state
                            self.current_joint_angles = list(joint_angles)
                            self.current_position = list(position)
                            
                            # Track update frequency
                            self.update_frequency.add_event()
                            
                    except json.JSONDecodeError:
                        self.log(f"[FB Error] Could not decode JSON: {line}")
                    except Exception as e:
                        self.log(f"[FB Error] {e}")
                        
            except socket.timeout:
                self.log("[FEEDBACK_DAEMON] Socket timeout. No data received.")
                continue # Don't disconnect on timeout
            except (socket.error, BrokenPipeError) as e:
                self.log(f"[FEEDBACK_DAEMON] Feedback connection error: {e}")
                self.connected = False
                break
            except Exception as e:
                self.log(f"[FEEDBACK_DAEMON] An unexpected error occurred in feedback handler: {e}")
                self.connected = False
                break
    
    def get_current_position(self):
        """Get the current position calculated from feedback"""
        return self.current_position.copy()
    
    def get_current_joint_angles(self):
        """Get the current joint angles from feedback"""
        return self.current_joint_angles.copy()
    
    def get_update_frequency(self):
        """Get the current update frequency in Hz"""
        return self.update_frequency.get_frequency()
    
    def is_connected(self):
        """Check if daemon is connected to tester"""
        return self.connected
    
    def disconnect(self):
        """Disconnect from tester"""
        self.connected = False
        if self.cmd_sock:
            self.cmd_sock.close()
        if self.fb_sock:
            self.fb_sock.close()
        self.log("[FEEDBACK_DAEMON] Disconnected from tester")

# Example usage and testing
if __name__ == "__main__":
    def log_callback(message):
        print(f"[LOG] {message}")
    
    # Create daemon instance
    daemon = FeedbackDaemon(tester_host='localhost', log_callback=log_callback)
    
    # Wait for connection
    time.sleep(2)
    
    # Test some positions
    test_positions = [
        (150, 0, 300, 0, 90, 0),
        (200, 100, 250, 30, 60, 45),
        (120, -50, 400, -15, 45, 90),
    ]
    
    try:
        for i, pos in enumerate(test_positions):
            print(f"\n--- Test Position {i+1} ---")
            success = daemon.process_target(*pos)
            if success:
                time.sleep(1)  # Wait for feedback
                current_pos = daemon.get_current_position()
                current_joints = daemon.get_current_joint_angles()
                freq = daemon.get_update_frequency()
                
                print(f"Current Position: {[round(p, 2) for p in current_pos]}")
                print(f"Current Joints: {[round(j, 2) for j in current_joints]}")
                print(f"Update Frequency: {freq:.1f} Hz")
            else:
                print("Failed to process target")
            
            time.sleep(2)
        
        # Keep running to show continuous updates
        print("\nContinuous monitoring (Ctrl+C to stop)...")
        while True:
            time.sleep(5)
            if daemon.is_connected():
                freq = daemon.get_update_frequency()
                print(f"Update Frequency: {freq:.1f} Hz")
            else:
                print("Daemon disconnected")
                break
                
    except KeyboardInterrupt:
        print("\nShutting down...")
        daemon.disconnect()
