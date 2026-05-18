# Current Runtime And Dataflow

Snapshot date: 2026-05-17

## Runtime Overview

The runtime model combines a Qt UI thread with background worker threads for control, hardware communication, pendant connectivity, and kinematics. The design intent is clear:

- the UI thread remains responsive,
- timing-sensitive motion work happens off the main thread,
- kinematics stays in a computation-oriented layer,
- hardware access stays in the hardware layer.

## Startup Flow

### Application startup

1. `launcher.py` creates `QApplication`.
2. `ui.main.MainWindow` is instantiated.
3. The UI builds widgets, panels, and the 3D visualizer.
4. Background threads are created and connected via Qt signals.
5. The GUI begins receiving feedback, status, and visualization updates.

### Visualizer startup

The `Robot3DVisualizer` widget initializes:

- OpenGL view widget
- grid and axis markers
- link and joint visuals
- TCP marker
- path and recent-path trail items
- playback timers and render throttling

The visualizer is not passive. It maintains playback state, path caches, recent trails, and metadata-driven rendering behavior.

## Thread Model

### UI thread

Responsibilities:

- widget event handling
- user commands
- display updates
- log output
- file dialogs
- table edits
- status presentation

### Control thread

The control thread runs at 100 Hz.

Per-loop responsibilities:

1. Read current cached joints.
2. Dequeue the next command without blocking.
3. Normalize command intent.
4. Compute next joint targets from interpolation state.
5. Compute Cartesian state via FK.
6. Emit joint targets to hardware.
7. Maintain loop cadence at 10 ms.

This is the core motion-execution loop.

### Hardware thread

Responsibilities:

- receive target joints from the control thread
- send them to the robot hardware interface
- read hardware feedback
- forward current state to the UI and other consumers

This thread isolates robot transport details from the control loop.

### Pendant thread

Responsibilities:

- scan matching serial ports
- issue pendant handshake probe
- verify the handshake response
- read pendant command strings
- emit command strings to the UI

The pendant is handled as a separate serial channel, which is the correct design choice if controller and pendant traffic may diverge later.

### Kinematics worker thread

Responsibilities:

- execute FK and IK without blocking the UI
- handle expensive planning-related conversions
- support responsive user interaction during teaching and validation

## Command Flow

## Jog flow

Typical jog sequence:

1. User presses a jog control in the GUI.
2. UI logic decides whether the action is a tap or continuous hold.
3. The request is converted into joint or Cartesian motion intent.
4. The control layer receives or derives targets.
5. The control thread interpolates toward the next joint target.
6. The hardware thread emits the joint payload to the robot interface.
7. Feedback returns and the UI updates position and status.

This flow means the control thread, not the UI, is the correct place to reason about execution timing.

## Program authoring flow

1. User teaches or inserts positions into the program table.
2. Each row is represented as a `ProgramRow`.
3. Rows may include pose, joint values, speed, acceleration, blend, and weld metadata.
4. The program is saved to JSON.

The JSON program is the editable planning artifact.

## Path generation flow

1. A program JSON file is loaded.
2. The path generator extracts valid motion rows.
3. Rows are grouped by motion type.
4. Interpolation is performed according to LINEAR, CURVE, or P2P behavior.
5. Redundant points are removed.
6. Weaving offsets are applied when weld parameters require it.
7. IK converts Cartesian points into joint-space points.
8. Joint limits and path sanity are checked.
9. The result is written as a trajectory file with machine metadata.

This is one of the most important end-to-end flows in the repository because it bridges human-authored programs and executable robot motion.

## Trajectory playback flow

1. The GUI loads a generated trajectory file.
2. Cartesian metadata and joint rows are parsed together when available.
3. Playback timers advance through the trajectory.
4. The visualizer updates robot pose and path overlays.
5. The control and hardware layers can drive actual execution depending on runtime mode.

## Visualization flow

The visualization layer has two overlapping responsibilities:

- show current robot state
- show path context for validation

### Live state display

The viewer draws:

- robot links
- joints
- TCP marker
- TCP orientation axes
- current TCP history

### Playback path display

When Cartesian trajectory metadata is present, the viewer can color and annotate the path using semantic information from the trajectory rows.

Recent behavior includes weld-aware path coloring:

- weld ON segments render in yellow
- weld OFF segments render in green

That makes the visualizer a semantic debugging tool rather than only a geometry display.

## File Formats And Data Movement

### JSON program files

Used for editable robot programs.

Typical fields per row:

- index
- motion type
- pose
- joint angles
- speed
- acceleration
- blend
- weld parameters
- comment

### KSM trajectory files

Used for machine-bound or playback-ready trajectories.

These files include:

- kinematics signature metadata
- joint angles
- motion phase
- Cartesian pose
- weld on or off state
- weave pattern information
- nominal position fields when available

### Serial joint stream

The hardware interface sends joint targets as newline-terminated ASCII CSV values. This is intentionally simple and easy to inspect, though it is not a strongly framed binary protocol.

### Pendant command stream

The pendant channel sends command strings after a successful serial handshake. The GUI interprets those strings using fixed character positions for jogging and action commands.

## Port Detection And Handshake Flow

The hardware and pendant layers both use pragmatic serial-port scanning patterns.

Candidate patterns include:

- `/dev/tty.usb*`
- `/dev/ttyUSB*`
- `/dev/ttyACM*`
- `COM*`

Pendant handshake sequence:

1. Open candidate serial port.
2. Wait briefly for device stabilization.
3. send `test_pendant` plus newline.
4. expect `pendant_ready` plus newline.
5. mark the port as active if the response matches exactly.

This is a simple but effective device-discovery mechanism.

## Current Operational Boundaries

### Best isolated responsibilities

- planning math in `planning/`
- serial abstraction in `hardware/robot_interface.py`
- control timing in `control/threads.py`

### Most coupled area

- `ui/main.py`

This file knows about many runtime concerns and acts as the system’s practical integration hub.

## Summary

The current runtime is best understood as a desktop robotics control station with:

- a large but capable Qt front end,
- a timing-oriented control layer,
- a fairly clean computational planning layer,
- pragmatic serial hardware integration,
- and a growing set of operational metadata in its generated motion files.
