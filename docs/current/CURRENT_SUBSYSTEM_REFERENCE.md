# Current Subsystem Reference

Snapshot date: 2026-05-17

## Top-Level Entry Points

### `launcher.py`

Primary application launcher. Creates the Qt application and opens `MainWindow` from `ui/main.py`.

### `main.py`

Historical root-level application entrypoint. Useful as legacy context, but the layered `ui/main.py` path is the more current implementation surface.

### `run.sh`

Shell launch helper for local execution.

## UI Layer

### `ui/main.py`

Primary UI and orchestration module.

Responsibilities:

- Main window construction.
- Toolbar and mode controls.
- Jogging controls.
- Teach-mode interactions.
- Program table operations.
- File loading and saving.
- 3D robot visualization.
- Thread initialization and signal wiring.
- Pendant command integration.
- Status and log presentation.

Important embedded components:

- `MainWindow`
- `Robot3DVisualizer`
- Qt timers for UI update cadence
- multiple slots tied to hardware, control, kinematics, and playback

Assessment:

This file is the most important operational surface in the project and also the largest concentration of maintenance risk.

### `ui/program_table_model.py`

Qt table model for program rows.

Responsibilities:

- Program-row display.
- Row formatting by motion type.
- UI-facing data projection for `ProgramRow` objects.

### `ui/settings_dialog.py`

Settings dialog and UI configuration support.

### `ui/yaskawa_3R.py` and `ui/yaskawa_3r_quat_test.py`

Likely experimental or model-specific visualization or test files. They are not the main control surface.

## Control Layer

### `control/threads.py`

Core runtime coordination module.

Key classes and responsibilities:

- `ControlThread`
  - Runs at 100 Hz.
  - Reads current state.
  - Dequeues commands without blocking.
  - Interpolates toward joint targets.
  - Emits joint targets to hardware.
  - Emits Cartesian state updates.
  - Owns movement progress state.

- `HardwareThread`
  - Bridges the control loop to the robot hardware interface.
  - Manages feedback reads and target transmission.

Assessment:

This file is the runtime heart of the system. Any change to motion execution behavior should be evaluated here first.

### `control/command_queue.py`

Small, explicit lock-based queue for control commands.

Responsibilities:

- Accept high-level commands.
- Provide non-blocking dequeue for the control loop.
- Offer compatibility methods for existing call sites.

### `control/state_machine.py`

Centralized control-state model.

States:

- `IDLE`
- `JOG`
- `EXECUTE`
- `ABORT`
- `RETURN_HOME`

Responsibilities:

- Define allowed transitions.
- Keep state updates explicit and thread-safe.

### `control/kin_worker.py`

Background kinematics worker used to keep UI interactions responsive.

### `control/jog_preplanner.py`

Appears to be a support or experimental component. Its role is less central than `threads.py` and should be reviewed before future expansion.

## Planning Layer

### `planning/kinematics_adapter.py`

Current canonical pose-conversion boundary.

Responsibilities:

- Translate between GUI pose convention and backend engine convention.
- Provide `solve_fk_gui`.
- Provide `solve_ik_gui`.
- Provide visual-chain position extraction for the 3D viewer.
- Publish a kinematics signature used by KSM metadata validation.

This file is one of the most important pieces in the repository because it prevents convention drift across UI, planning, and generated data.

### `planning/kin_serial.py`

Backend kinematics engine wrapper.

Responsibilities:

- Load or expose the underlying kinematics engine.
- Support direct FK and IK use below the GUI-facing adapter.

### `planning/path_generator.py`

Trajectory generation engine.

Responsibilities:

- Load program JSON.
- Extract and validate waypoints.
- Group waypoints by motion type.
- Interpolate LINEAR, CURVE, and P2P segments.
- Deduplicate near-identical points.
- Apply weave overlays.
- Run IK conversion.
- Validate joint limits.
- Write servo-ready output with phase and weld metadata.

This is the key bridge from taught program data to executable robot motion files.

### `planning/data_models.py`

Domain model definitions.

Core types:

- `Pose`
- `WeldParams`
- `ProgramRow`
- `RobotState`
- `ProgramManager`

### `planning/visual_kinematics/`

Vendored or project-local robotics math support library used by the kinematics stack.

### `planning/kin_serial_yaskawa.py`

Alternate or model-specific kinematics module. Appears specialized rather than general-purpose.

## Hardware Layer

### `hardware/robot_interface.py`

Hardware abstraction boundary.

Key classes:

- `RobotHardwareInterface`
- `MockRobotHardwareInterface`
- `SerialRobotHardwareInterface`

Responsibilities:

- Connect and disconnect.
- Send joint targets.
- Read hardware feedback.
- Hide serial details from upper layers.

### `hardware/pendant_class.py`

Pendant auto-detection and command reception thread.

Responsibilities:

- Scan candidate ports.
- Perform handshake using `test_pendant` probe logic.
- Expect `pendant_ready` response.
- Maintain connection state.
- Emit command strings back to the UI.

### `hardware/feedback_daemon.py` and `hardware/enhanced_feedback_daemon.py`

Additional hardware feedback paths. These likely reflect incremental evolution of the hardware subsystem and should be reviewed for overlap with `HardwareThread`.

### `hardware/commander.py`

Motor or auxiliary command support module used by the control layer.

## Data and Content Directories

### `positions/`

Operational content store for programs and generated motion examples.

Contents include:

- saved JSON programs
- generated trajectories
- shape or text-writing examples
- experiment-specific motion sets

### `logs/`

Runtime logging output.

### `_archive_cleanup_20260315/`

Archived material retained for historical or recovery purposes.

## Legacy and Compatibility Surface

The project root still includes historical single-file versions such as:

- `kin_serial.py`
- `path_generator.py`
- `program_table_model.py`
- `data_models.py`

These are important for migration history, but contributors should prefer the layered modules under `planning/`, `control/`, `hardware/`, and `ui/` unless a specific compatibility reason exists.

## Supporting Non-Core Files

### `AR4_teensy41_sketch_v6.6.cpp`

Firmware-side logic or controller integration reference for the robot hardware. This matters when Python-side serial behavior must match embedded expectations.

### experimental scripts in the root

Files like motor runners, speed experiments, or IK benchmarks are useful engineering utilities but are not the primary runtime path.

## Subsystem Ownership Summary

- If the issue is visual or user-facing, start in `ui/`.
- If the issue is timing, interpolation, or execution state, start in `control/`.
- If the issue is FK, IK, motion geometry, or output generation, start in `planning/`.
- If the issue is serial communication, connection scanning, pendant behavior, or robot IO, start in `hardware/`.
