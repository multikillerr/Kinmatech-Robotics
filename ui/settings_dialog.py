from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel, 
    QPushButton, QDoubleSpinBox, QSpinBox, QSlider, QDialogButtonBox
)
from PyQt6.QtCore import Qt

COLOR_SCHEMA = {
    "primary": "#1976D2",
    "secondary": "#7B1FA2",
    "success": "#388E3C",
    "danger": "#C63D3D",
    "warning": "#E29613",
    "info": "#3092C7",
    "background": "#B5CBE0",
    "surface": "#FFFFFF",
    "text": "#263238",
    "text_secondary": "#546E7A",
    "border": "#B0BEC5",
    "border_light": "#CFD8DC",
}

class ModernButton(QPushButton):
    def __init__(self, text, color="default"):
        super().__init__(text)
        self.setMinimumHeight(35)
        color_map = {
            "primary": COLOR_SCHEMA["primary"],
            "secondary": COLOR_SCHEMA["secondary"],
            "success": COLOR_SCHEMA["success"],
            "danger": COLOR_SCHEMA["danger"],
            "warning": COLOR_SCHEMA["warning"],
            "info": COLOR_SCHEMA["info"],
        }
        btn_color = color_map.get(color, COLOR_SCHEMA["surface"])
        text_color = COLOR_SCHEMA["text"] if color == "default" else "#fff"
        self.setStyleSheet(f"""
            QPushButton {{
                border: 2px solid {COLOR_SCHEMA['border']};
                border-radius: 6px;
                background-color: {btn_color};
                color: {text_color};
                padding: 5px;
            }}
            QPushButton:hover {{
                border-color: {COLOR_SCHEMA['border_light']};
            }}
        """)

class AdvancedSettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_gui = parent
        self.setWindowTitle("Advanced Settings")
        self.setModal(True)
        self.resize(500, 600)
        
        # Store original values for cancel functionality
        self.original_values = {}
        
        self.init_ui()
        self.load_current_values()
        
    def init_ui(self):
        """Initialize the settings dialog UI"""
        layout = QVBoxLayout(self)
        
        # Path Generation Settings
        path_group = QGroupBox("Path Generation Settings")
        path_group.setStyleSheet(f"""
            QGroupBox {{
                background-color: {COLOR_SCHEMA['surface']};
                border: 1px solid {COLOR_SCHEMA['border']};
                border-radius: 6px;
                color: {COLOR_SCHEMA['text_secondary']};
                font-weight: bold;
                margin-top: 0.5em;
                padding-top: 0.5em;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }}
        """)
        path_layout = QGridLayout()
        
        # Stabilization Delay
        path_layout.addWidget(QLabel("Stabilization Delay (s):"), 0, 0)
        self.stabilization_delay_input = QDoubleSpinBox()
        self.stabilization_delay_input.setMinimum(0)
        self.stabilization_delay_input.setMaximum(60)
        self.stabilization_delay_input.setDecimals(2)
        self.stabilization_delay_input.setValue(0.5)
        path_layout.addWidget(self.stabilization_delay_input, 0, 1)
        
        # Look-ahead buffer
        path_layout.addWidget(QLabel("Look-Ahead Buffer:"), 1, 0)
        self.look_ahead_input = QSpinBox()
        self.look_ahead_input.setMinimum(1)
        self.look_ahead_input.setMaximum(100)
        self.look_ahead_input.setValue(10)
        path_layout.addWidget(self.look_ahead_input, 1, 1)
        
        # Max Acceleration
        path_layout.addWidget(QLabel("Max Acceleration:"), 2, 0)
        self.max_acceleration_input = QDoubleSpinBox()
        self.max_acceleration_input.setMinimum(1.0)
        self.max_acceleration_input.setMaximum(1000.0)
        self.max_acceleration_input.setValue(100.0)
        path_layout.addWidget(self.max_acceleration_input, 2, 1)
        
        # Junction Deviation
        path_layout.addWidget(QLabel("Junction Deviation:"), 3, 0)
        self.junction_deviation_input = QDoubleSpinBox()
        self.junction_deviation_input.setMinimum(0.01)
        self.junction_deviation_input.setMaximum(10.0)
        self.junction_deviation_input.setDecimals(3)
        self.junction_deviation_input.setValue(0.1)
        path_layout.addWidget(self.junction_deviation_input, 3, 1)
        
        path_group.setLayout(path_layout)
        layout.addWidget(path_group)
        
        # Timer Settings
        timer_group = QGroupBox("Timer Settings")
        timer_group.setStyleSheet(path_group.styleSheet())
        timer_layout = QGridLayout()
        
        # Delay Timer
        timer_layout.addWidget(QLabel("Default Delay Timer (s):"), 0, 0)
        self.timer_input = QDoubleSpinBox()
        self.timer_input.setMinimum(0)
        self.timer_input.setMaximum(3600)
        self.timer_input.setDecimals(3)
        self.timer_input.setValue(0)
        timer_layout.addWidget(self.timer_input, 0, 1)
        
        timer_group.setLayout(timer_layout)
        layout.addWidget(timer_group)
        
        # Communication Settings
        comm_group = QGroupBox("Communication Settings")
        comm_group.setStyleSheet(path_group.styleSheet())
        comm_layout = QGridLayout()
        
        # Connection Timeout
        comm_layout.addWidget(QLabel("Connection Timeout (s):"), 0, 0)
        self.connection_timeout_input = QDoubleSpinBox()
        self.connection_timeout_input.setMinimum(1.0)
        self.connection_timeout_input.setMaximum(30.0)
        self.connection_timeout_input.setValue(5.0)
        comm_layout.addWidget(self.connection_timeout_input, 0, 1)
        
        # Retry Attempts
        comm_layout.addWidget(QLabel("Retry Attempts:"), 1, 0)
        self.retry_attempts_input = QSpinBox()
        self.retry_attempts_input.setMinimum(1)
        self.retry_attempts_input.setMaximum(10)
        self.retry_attempts_input.setValue(3)
        comm_layout.addWidget(self.retry_attempts_input, 1, 1)
        
        # Feedback Frequency
        comm_layout.addWidget(QLabel("Feedback Update Rate (Hz):"), 2, 0)
        self.feedback_rate_input = QSpinBox()
        self.feedback_rate_input.setMinimum(1)
        self.feedback_rate_input.setMaximum(100)
        self.feedback_rate_input.setValue(10)
        comm_layout.addWidget(self.feedback_rate_input, 2, 1)
        
        comm_group.setLayout(comm_layout)
        layout.addWidget(comm_group)
        
        # Safety Settings
        safety_group = QGroupBox("Safety Settings")
        safety_group.setStyleSheet(path_group.styleSheet())
        safety_layout = QGridLayout()
        
        # Position Tolerance
        safety_layout.addWidget(QLabel("Position Tolerance (mm):"), 0, 0)
        self.position_tolerance_input = QDoubleSpinBox()
        self.position_tolerance_input.setMinimum(0.01)
        self.position_tolerance_input.setMaximum(10.0)
        self.position_tolerance_input.setDecimals(3)
        self.position_tolerance_input.setValue(0.1)
        safety_layout.addWidget(self.position_tolerance_input, 0, 1)
        
        # Joint Tolerance
        safety_layout.addWidget(QLabel("Joint Tolerance (°):"), 1, 0)
        self.joint_tolerance_input = QDoubleSpinBox()
        self.joint_tolerance_input.setMinimum(0.01)
        self.joint_tolerance_input.setMaximum(5.0)
        self.joint_tolerance_input.setDecimals(3)
        self.joint_tolerance_input.setValue(0.07)
        safety_layout.addWidget(self.joint_tolerance_input, 1, 1)
        
        # Emergency Stop Timeout
        safety_layout.addWidget(QLabel("Emergency Stop Timeout (s):"), 2, 0)
        self.emergency_timeout_input = QDoubleSpinBox()
        self.emergency_timeout_input.setMinimum(1.0)
        self.emergency_timeout_input.setMaximum(60.0)
        self.emergency_timeout_input.setValue(10.0)
        safety_layout.addWidget(self.emergency_timeout_input, 2, 1)
        
        safety_group.setLayout(safety_layout)
        layout.addWidget(safety_group)
        
        # Action Buttons
        button_layout = QHBoxLayout()
        
        self.reset_button = ModernButton("Reset to Defaults", "warning")
        self.reset_button.clicked.connect(self.reset_to_defaults)
        button_layout.addWidget(self.reset_button)
        
        button_layout.addStretch()
        
        # Standard dialog buttons
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept_changes)
        self.button_box.rejected.connect(self.reject_changes)
        button_layout.addWidget(self.button_box)
        
        layout.addLayout(button_layout)
        
    def load_current_values(self):
        """Load current values from the main GUI"""
        if not self.parent_gui:
            return
            
        # Store original values
        self.original_values = {
            'stabilization_delay': getattr(self.parent_gui, 'stabilization_delay', 0.5),
            'look_ahead_buffer_size': getattr(self.parent_gui, 'look_ahead_buffer_size', 10),
            'max_acceleration': getattr(self.parent_gui, 'max_acceleration', 100.0),
            'junction_deviation': getattr(self.parent_gui, 'junction_deviation', 0.1),
            'timer_default': 0.0,
            'connection_timeout': 5.0,
            'retry_attempts': 3,
            'feedback_rate': 10,
            'position_tolerance': 0.1,
            'joint_tolerance': 0.07,
            'emergency_timeout': 10.0
        }
        
        # Load values into controls
        self.stabilization_delay_input.setValue(self.original_values['stabilization_delay'])
        self.look_ahead_input.setValue(self.original_values['look_ahead_buffer_size'])
        self.max_acceleration_input.setValue(self.original_values['max_acceleration'])
        self.junction_deviation_input.setValue(self.original_values['junction_deviation'])
        self.timer_input.setValue(self.original_values['timer_default'])
        self.connection_timeout_input.setValue(self.original_values['connection_timeout'])
        self.retry_attempts_input.setValue(self.original_values['retry_attempts'])
        self.feedback_rate_input.setValue(self.original_values['feedback_rate'])
        self.position_tolerance_input.setValue(self.original_values['position_tolerance'])
        self.joint_tolerance_input.setValue(self.original_values['joint_tolerance'])
        self.emergency_timeout_input.setValue(self.original_values['emergency_timeout'])
        
    def reset_to_defaults(self):
        """Reset all settings to default values"""
        self.stabilization_delay_input.setValue(0.5)
        self.look_ahead_input.setValue(10)
        self.max_acceleration_input.setValue(100.0)
        self.junction_deviation_input.setValue(0.1)
        self.timer_input.setValue(0.0)
        self.connection_timeout_input.setValue(5.0)
        self.retry_attempts_input.setValue(3)
        self.feedback_rate_input.setValue(10)
        self.position_tolerance_input.setValue(0.1)
        self.joint_tolerance_input.setValue(0.07)
        self.emergency_timeout_input.setValue(10.0)
        
    def accept_changes(self):
        """Apply changes and close dialog"""
        if self.parent_gui:
            # Update main GUI values
            self.parent_gui.stabilization_delay = self.stabilization_delay_input.value()
            self.parent_gui.look_ahead_buffer_size = self.look_ahead_input.value()
            self.parent_gui.max_acceleration = self.max_acceleration_input.value()
            self.parent_gui.junction_deviation = self.junction_deviation_input.value()
            
            # Update other GUI elements if they exist
            if hasattr(self.parent_gui, 'stabilization_delay_input'):
                self.parent_gui.stabilization_delay_input.setValue(self.stabilization_delay_input.value())
            if hasattr(self.parent_gui, 'look_ahead_input'):
                self.parent_gui.look_ahead_input.setValue(self.look_ahead_input.value())
            
            # Save to configuration file
            self.save_settings()
            
            # Log the changes
            if hasattr(self.parent_gui, 'console'):
                self.parent_gui.console.append("Advanced settings updated successfully")
                
        self.accept()
        
    def reject_changes(self):
        """Cancel changes and close dialog"""
        self.reject()
        
    def save_settings(self):
        """Save settings to configuration file"""
        try:
            # Ensure config directory exists
            config_dir = "config"
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)
            
            settings = {
                'stabilization_delay': self.stabilization_delay_input.value(),
                'look_ahead_buffer_size': self.look_ahead_input.value(),
                'max_acceleration': self.max_acceleration_input.value(),
                'junction_deviation': self.junction_deviation_input.value(),
                'timer_default': self.timer_input.value(),
                'connection_timeout': self.connection_timeout_input.value(),
                'retry_attempts': self.retry_attempts_input.value(),
                'feedback_rate': self.feedback_rate_input.value(),
                'position_tolerance': self.position_tolerance_input.value(),
                'joint_tolerance': self.joint_tolerance_input.value(),
                'emergency_timeout': self.emergency_timeout_input.value()
            }
            
            with open("config/advanced_settings.json", "w") as f:
                json.dump(settings, f, indent=4)
                
        except Exception as e:
            if hasattr(self.parent_gui, 'console'):
                self.parent_gui.console.append(f"Failed to save advanced settings: {e}")
                
    def load_settings(self):
        """Load settings from configuration file"""
        try:
            if os.path.exists("config/advanced_settings.json"):
                with open("config/advanced_settings.json", "r") as f:
                    settings = json.load(f)
                
                # Apply loaded settings
                self.stabilization_delay_input.setValue(settings.get('stabilization_delay', 0.5))
                self.look_ahead_input.setValue(settings.get('look_ahead_buffer_size', 10))
                self.max_acceleration_input.setValue(settings.get('max_acceleration', 100.0))
                self.junction_deviation_input.setValue(settings.get('junction_deviation', 0.1))
                self.timer_input.setValue(settings.get('timer_default', 0.0))
                self.connection_timeout_input.setValue(settings.get('connection_timeout', 5.0))
                self.retry_attempts_input.setValue(settings.get('retry_attempts', 3))
                self.feedback_rate_input.setValue(settings.get('feedback_rate', 10))
                self.position_tolerance_input.setValue(settings.get('position_tolerance', 0.1))
                self.joint_tolerance_input.setValue(settings.get('joint_tolerance', 0.07))
                self.emergency_timeout_input.setValue(settings.get('emergency_timeout', 10.0))
                
        except Exception as e:
            if hasattr(self.parent_gui, 'console'):
                self.parent_gui.console.append(f"Failed to load advanced settings: {e}")
