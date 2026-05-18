# Kinmatech Robotics Control Application

A professional-grade PyQt6-based control interface for 6-DOF robotic arm operations. The system combines interactive GUI controls, deterministic motion execution, offline program generation, and real-time 3D visualization into a unified desktop application.

## What It Does

- **Interactive Control**: Jog the robot in joint or Cartesian space via mouse, keyboard, or external pendant.
- **Teaching**: Record positions by moving the robot and storing snapshots into a program table.
- **Program Authoring**: Create, edit, and save robot motion programs as structured JSON files.
- **Path Generation**: Convert taught programs into servo-ready trajectories with motion phases, weaving patterns, and kinematic validation.
- **Visualization**: Real-time 3D viewer showing robot pose, path overlays, and motion semantics (weld state, motion type).
- **Execution**: Play back generated programs with precise timing and feedback display.
- **Hardware Integration**: Serial communication with robot controllers and support for external control pendants.

## Quick Start

### Prerequisites

- Python 3.8+
- Virtual environment (recommended)
- Robot hardware connected via serial port (optional for simulation)

### Installation

1. Clone or download the repository:
```bash
cd kinmatech_robotics_final
```

2. Create and activate a virtual environment:
```bash
python3 -m venv .venv
source .venv/bin/activate  # macOS/Linux
# or
.venv\Scripts\activate  # Windows
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

### Running the Application

```bash
python launcher.py
```

Or directly:
```bash
python ui/main.py
```

The application will launch with the main window showing the jog controls, status panel, program table, and 3D visualization.

## Project Structure

```
├── ui/                          # Desktop GUI layer (PyQt6)
│   ├── main.py                  # Main application window
│   ├── program_table_model.py   # Program table display logic
│   └── settings_dialog.py       # Settings and configuration
│
├── control/                     # Control orchestration layer
│   ├── threads.py               # 100 Hz control loop & hardware thread
│   ├── command_queue.py         # Thread-safe command queue
│   ├── state_machine.py         # Motion state management
│   └── kin_worker.py            # Kinematics computation worker
│
├── planning/                    # Planning & computation layer
│   ├── kinematics_adapter.py    # FK/IK interface (GUI↔backend)
│   ├── path_generator.py        # Trajectory generation
│   ├── data_models.py           # Program and robot state definitions
│   ├── kin_serial.py            # Kinematics engine wrapper
│   └── visual_kinematics/       # Robot math & DH parameters
│
├── hardware/                    # Hardware communication layer
│   ├── robot_interface.py       # Serial communication abstraction
│   ├── pendant_class.py         # External pendant integration
│   ├── feedback_daemon.py       # Feedback monitoring
│   └── commander.py             # Motor control interface
│
├── positions/                   # Saved programs & trajectories
│   ├── test/                    # Example programs
│   ├── capability_montage/      # Shape demonstration
│   └── ritesh/, kinmatech/      # Text-writing examples
│
├── docs/current/                # Current-state documentation
│   ├── README.md                # Doc navigation & overview
│   ├── CURRENT_PROJECT_ANALYSIS.md
│   ├── CURRENT_SUBSYSTEM_REFERENCE.md
│   ├── CURRENT_RUNTIME_AND_DATAFLOW.md
│   └── CURRENT_TECHNICAL_DEBT.md
│
├── launcher.py                  # Application entry point
├── requirements.txt             # Python dependencies
└── run.sh                       # Shell launch script
```

## Key Features

### Jogging & Teach Mode

- **Joint Space**: Rotate individual joints with independent control
- **Cartesian Space**: Move the tool point in X, Y, Z, roll, pitch, yaw
- **Pendant Support**: External wireless or wired control pendant integration
- **Speed Control**: Adjustable movement velocity (1–100+ deg/s)

### Program Management

- **Table-Based Editing**: Add, delete, reorder motion waypoints
- **Motion Types**: Point-to-point (P2P), linear interpolation, curved paths
- **Welding Parameters**: Attach weld state, power, wire feed, and weaving patterns to each segment
- **Save/Load**: Programs stored as JSON for easy backup and sharing

### Path Generation

- **Automatic IK**: Converts Cartesian waypoints to joint-space trajectories
- **Weaving Overlay**: Applies sine, triangular, zigzag, or figure-8 weaving patterns
- **Motion Phasing**: Assigns ACCEL, CRUISE, DECEL profiles for smooth motion
- **Validation**: Joint limit checking and path sanity verification

### Visualization

- **3D Robot Model**: Real-time OpenGL display of all 6 joints and TCP
- **Path Preview**: Shows upcoming trajectory in the workspace
- **Semantic Coloring**: Path segments colored by weld state (yellow=on, green=off)
- **Live Feedback**: Current position and orientation displayed
- **Playback**: Step through or play back trajectories with adjustable speed

## Documentation

For detailed information, see the **[docs/current/README.md](docs/current/README.md)** which includes:

- **[CURRENT_PROJECT_ANALYSIS.md](docs/current/CURRENT_PROJECT_ANALYSIS.md)** – System overview and architecture
- **[CURRENT_SUBSYSTEM_REFERENCE.md](docs/current/CURRENT_SUBSYSTEM_REFERENCE.md)** – Module-by-module code reference
- **[CURRENT_RUNTIME_AND_DATAFLOW.md](docs/current/CURRENT_RUNTIME_AND_DATAFLOW.md)** – Execution flow and threading model
- **[CURRENT_TECHNICAL_DEBT.md](docs/current/CURRENT_TECHNICAL_DEBT.md)** – Known issues and refactoring priorities

## System Requirements

### Hardware

- **Robot**: Yaskawa-based 6-DOF arm (AR4 or similar) with Teensy 4.1 controller
- **Serial Port**: For robot communication (115200 baud)
- **Pendant** (Optional): Separate serial port for wireless/wired control pendant

### Software

- macOS 10.14+, Linux (Ubuntu 18.04+), or Windows 10+
- Python 3.8 or later
- PyQt6, PyOpenGL, numpy, scipy, pyserial

### Display

- 1920×1080 minimum recommended (3-column layout + visualization)
- GPU recommended for smooth 3D rendering

## Usage Workflow

### Teaching a Program

1. Click **[Teach]** button to enter teach mode
2. Use jog controls to position the robot
3. Click **[Add Position]** to record a waypoint
4. Repeat for all waypoints
5. Save the program: **File → Save Program**

### Generating a Trajectory

1. Load a saved program: **File → Load Program**
2. Click **[Generate Path]** to compute joint angles and motion phases
3. The trajectory is saved as a `.ksm` file in the positions directory
4. Review the 3D preview to verify the path

### Executing a Program

1. Load a trajectory: **File → Load Trajectory**
2. Click **[Play]** to begin execution (or **[Dry Run]** for visualization-only)
3. Use the speed slider to control playback velocity
4. Click **[Stop]** to halt at any time

### Using a Pendant

1. Connect the pendant over serial (auto-detected)
2. The status bar shows **Pendant: [port]** when connected
3. Use pendant jogging buttons for smooth manual control
4. Pendant commands are processed in real-time

## Control Loop

The application runs a **deterministic 100 Hz control loop** that:

1. Reads current robot state
2. Dequeues the next user command
3. Interpolates toward target joint angles
4. Computes forward kinematics
5. Sends updated targets to hardware
6. Updates UI and visualization

This ensures smooth, predictable motion independent of GUI refresh rates.

## File Formats

### Program Files (.json)

Human-editable programs with motion waypoints:

```json
{
  "filename": "example.json",
  "program_name": "my_program",
  "rows": [
    {
      "idx": 1,
      "type": "LINEAR",
      "pose": {"x": 500, "y": 600, "z": 800, "rx": 0, "ry": 0, "rz": 0},
      "joints_deg": [0, 45, -90, 0, 0, 0],
      "speed": 100,
      "accel": 50,
      "blend": 2.0,
      "weld": {"on": false, ...},
      "comment": "Approach position"
    }
  ]
}
```

### Trajectory Files (.ksm)

Machine-bound trajectories with metadata:

```
# KSM_MODEL=KINMATECH_ROBO_ARM_1.0
# KSM_VERSION=1.0
# KSM_DH=...
j1,j2,j3,j4,j5,j6,phase,x,y,z,rx,ry,rz,weld_on,...
0.0,45.0,-90.0,0.0,0.0,0.0,ACCEL,500.0,600.0,800.0,...
```

## Troubleshooting

### Application won't start

- Verify Python 3.8+: `python --version`
- Check dependencies: `pip install -r requirements.txt`
- On macOS, you may need to grant app permissions in System Preferences

### Robot not responding

- Check serial port: `ls /dev/tty.*` (macOS/Linux) or Device Manager (Windows)
- Verify baud rate is 115200
- Confirm robot power and USB connection
- Check `ui/main.py` for port scanning logic

### Visualization is slow

- Reduce trajectory preview detail
- Disable path history (Clear Path button)
- Check GPU/driver support for OpenGL

### Pendant not detected

- Verify pendant is connected and powered
- Check that the handshake probe uses `test_pendant` (see [hardware/pendant_class.py](hardware/pendant_class.py#L69))
- Pendant should respond with `pendant_ready` to be accepted

## Contributing

See [docs/current/CURRENT_TECHNICAL_DEBT.md](docs/current/CURRENT_TECHNICAL_DEBT.md) for guidance on:

- Priority refactoring areas
- Architectural guidelines
- Test and documentation standards

## License

Kinmatech Robotics — Internal Development

## Support & Questions

- **Code reference**: See [docs/current/CURRENT_SUBSYSTEM_REFERENCE.md](docs/current/CURRENT_SUBSYSTEM_REFERENCE.md)
- **Runtime questions**: See [docs/current/CURRENT_RUNTIME_AND_DATAFLOW.md](docs/current/CURRENT_RUNTIME_AND_DATAFLOW.md)
- **Troubleshooting**: See [docs/current/CURRENT_TECHNICAL_DEBT.md](docs/current/CURRENT_TECHNICAL_DEBT.md#troubleshooting-guide)
