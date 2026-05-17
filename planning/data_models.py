#!/usr/bin/env python3
"""
Data models for Kinmatech Robotics Control Application
Contains dataclasses for robot poses, welding parameters, program rows, and robot state.
"""

from dataclasses import dataclass, asdict
from typing import List, Optional, Union
import json


@dataclass
class Pose:
    """Robot pose with position (x,y,z) and rotation (rx,ry,rz)"""
    x: float
    y: float
    z: float
    rx: float  # rotation around x-axis
    ry: float  # rotation around y-axis
    rz: float  # rotation around z-axis
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Pose':
        """Create Pose from dictionary"""
        return cls(**data)


@dataclass
class WeldParams:
    """Welding parameters"""
    on: bool           # welding on/off
    power: float       # welding power percentage (0-100)
    wire_feed: float   # wire feed speed
    gas_pre: float     # pre-gas time in seconds
    gas_post: float    # post-gas time in seconds
    weaving_type: str  # weaving pattern type
    timer: int         # timer in seconds (0-999)
    sensing_trigger: str  # sensing trigger (None or 1-16)
    # Per-waypoint weave geometry (defaults match PathGenerator legacy behaviour)
    weave_amplitude: float = 3.0   # lateral half-width in mm
    weave_frequency: float = 2.5   # oscillations per second (Hz)
    weave_dwell: float = 0.0       # dwell time at pattern extremes (seconds)
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: dict) -> 'WeldParams':
        """Create WeldParams from dictionary.
        Handles old JSON files that lack the weave geometry fields."""
        # Supply defaults for fields added after initial release
        data.setdefault('weave_amplitude', 3.0)
        data.setdefault('weave_frequency', 2.5)
        data.setdefault('weave_dwell', 0.0)
        return cls(**data)


@dataclass
class ProgramRow:
    """Single row in a robot program"""
    idx: int                           # row index
    type: str                          # move type: "LINEAR", "JOINT", "CIRCULAR", etc.
    pose: Optional[Pose]              # target pose (None for joint moves)
    joints_deg: Optional[List[float]] # joint angles in degrees (None for pose moves)
    speed: float                      # movement speed (mm/s or deg/s)
    accel: float                      # acceleration (mm/s² or deg/s²)
    blend: float                      # blend radius for smooth transitions
    weld: Optional[WeldParams]        # welding parameters (None if no welding)
    comment: str                      # user comment
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        data = asdict(self)
        # Handle nested objects
        if self.pose is not None:
            data['pose'] = self.pose.to_dict()
        if self.weld is not None:
            data['weld'] = self.weld.to_dict()
        return data
    
    @classmethod
    def from_dict(cls, data: dict) -> 'ProgramRow':
        """Create ProgramRow from dictionary"""
        # Handle nested objects
        if data.get('pose') is not None:
            data['pose'] = Pose.from_dict(data['pose'])
        if data.get('weld') is not None:
            data['weld'] = WeldParams.from_dict(data['weld'])
        return cls(**data)


@dataclass
class RobotState:
    """Current robot state"""
    mode: str                    # "MANUAL", "AUTO", "TEACH", etc.
    joints_deg: List[float]     # current joint angles in degrees
    pose: Pose                  # current robot pose
    override_pct: float         # speed override percentage (0-100)
    fault: bool                 # fault condition present
    connected: bool             # connection status
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        data = asdict(self)
        data['pose'] = self.pose.to_dict()
        return data
    
    @classmethod
    def from_dict(cls, data: dict) -> 'RobotState':
        """Create RobotState from dictionary"""
        if data.get('pose') is not None:
            data['pose'] = Pose.from_dict(data['pose'])
        return cls(**data)


class ProgramManager:
    """Manages robot program serialization and deserialization"""
    
    @staticmethod
    def save_program(program_rows: List[ProgramRow], filename: str) -> None:
        """Save program rows to JSON file"""
        data = {
            'program': [row.to_dict() for row in program_rows],
            'version': '1.0',
            'created_by': 'Kinmatech Robotics Control'
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
    
    @staticmethod
    def load_program(filename: str) -> List[ProgramRow]:
        """Load program rows from JSON file"""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        program_data = data.get('program', [])
        return [ProgramRow.from_dict(row_data) for row_data in program_data]