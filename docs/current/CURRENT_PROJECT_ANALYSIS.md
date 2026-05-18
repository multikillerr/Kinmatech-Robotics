# Current Project Analysis

Snapshot date: 2026-05-17

## Executive Summary

Kinmatech Robotics is a desktop robot-control application centered on a PyQt6 GUI, a deterministic control loop, a planning layer for forward and inverse kinematics plus trajectory generation, and a hardware layer for serial communication with the robot and pendant. The project combines three distinct problem areas in one repository:

1. Interactive robot operation through a GUI.
2. Offline or semi-offline program authoring and trajectory generation.
3. Real-time or near-real-time command execution against robot hardware.

The repository shows evidence of active refactoring from an earlier monolithic layout into a layered design. The current code largely follows a four-layer structure, but several historical files and imports remain, so the architecture is best described as layered in intention and mostly layered in implementation.

## What The System Does

At a practical level, the system allows an operator or developer to:

- Connect to robot hardware over serial.
- Jog the robot in joint or Cartesian space.
- Teach positions into a program table.
- Save and load robot programs as JSON.
- Convert taught programs into servo-ready trajectory files.
- Visualize robot motion in a 3D OpenGL view.
- Run generated paths with motion and weld metadata.
- Integrate an external pendant over a dedicated serial connection.

## Technology Stack

The active Python stack in `requirements.txt` indicates the main implementation choices:

- PyQt6 for desktop GUI and threading primitives.
- pyqtgraph and PyOpenGL for 3D visualization.
- numpy and scipy for kinematics, interpolation, and orientation handling.
- pyserial for robot and pendant communication.
- visual-kinematics for the underlying robot-chain math.

This is a desktop engineering tool rather than a web service or distributed system.

## Architectural Shape

The current code is organized around four primary layers:

### UI layer

The UI layer is centered on `ui/main.py`. It owns the main window, widgets, status display, 3D visualization, button handlers, file dialogs, program-table interactions, and user-facing orchestration glue.

### Control layer

The control layer in `control/` contains the fixed-rate control thread, command queue, state machine, and background workers that mediate between user intent and robot motion. This is where timing-sensitive orchestration lives.

### Planning layer

The planning layer in `planning/` contains data models, kinematics adapters, FK and IK access, and path generation logic. This is the main computational layer.

### Hardware layer

The hardware layer in `hardware/` owns serial connection abstractions, pendant integration, and feedback or command channels to physical devices.

## Primary Runtime Entry Points

### Normal application launch

The main desktop entrypoint is `launcher.py`, which creates the Qt application and instantiates `ui.main.MainWindow`.

### Direct UI execution

The repository also supports running the UI directly via `ui/main.py`, which effectively acts as the application core during development.

### Path and program artifacts

The `positions/` tree acts as a working library of saved programs and generated motion data. It functions as operational content, not only as test data.

## Current Strengths

### Strong separation of computational logic from device IO

The planning layer isolates FK, IK, interpolation, metadata writing, and path generation from serial communication details. That separation makes the kinematics code more reusable and easier to test in isolation.

### Deterministic control-loop intent

The control layer explicitly implements a 100 Hz fixed-rate loop. This is one of the most technically important design choices in the repository because it provides a stable place to reason about command execution timing.

### Rich trajectory representation

Generated trajectories include not only joint positions but also pose, motion phase, weld state, weave data, and machine signature metadata. That gives the system a stronger foundation than a simple list of joint angles.

### Practical operator tooling

The GUI appears to be built around real usage: jogging, teaching, playback, visualization, weld state, and pendant support are integrated into one operator surface.

## Current Weaknesses

### Documentation drift

Several existing markdown files describe earlier snapshots of the system. They remain useful historically, but they should not be treated as exact descriptions of the current runtime.

### Large UI surface

`ui/main.py` remains a very large file that mixes presentation logic, event handling, orchestration, visualization behavior, and some coordination responsibilities. The project is functional, but maintenance cost is concentrated there.

### Multiple legacy paths and backups

The repository contains backup directories and older module copies. These are useful as migration history but they increase ambiguity about which code is authoritative.

### Mixed architectural purity

The repo has moved toward a clean layered model, but some practical shortcuts remain. This is normal in a migrating codebase, but it means contributors need to confirm behavior from code rather than relying entirely on architectural intent.

## Directory-Level Assessment

### `ui/`

High-value, high-change surface. This is the operational front end and the place where most user-visible behavior is expressed.

### `control/`

Operational core for timing, command dispatch, and motion-state transitions.

### `planning/`

Most reusable and conceptually clean subsystem. This layer contains the highest concentration of robotics-specific algorithmic logic.

### `hardware/`

Boundary layer for serial protocols and external device handling. This is where real-world variability and failure handling need to be strongest.

### `positions/`

Operational asset store containing programs and examples. It also acts as a de facto fixture library for testing generated motion content.

### root-level scripts

The root contains launch scripts, archived experiments, and historical compatibility files. Some are still useful; some should eventually be retired or moved into clearer folders.

## Data Model Assessment

The core program representation is based around `Pose`, `WeldParams`, `ProgramRow`, `RobotState`, and `ProgramManager` in `planning/data_models.py`.

This is a good foundation because:

- Robot programs have an explicit typed structure.
- Weld behavior is attached directly to motion rows.
- The same logical model can be serialized to JSON and later converted into trajectory files.

The main caveat is that there are multiple storage formats in the repo: JSON program files and generated KSM or CSV-like trajectory files. That dual-format model is justified operationally, but it needs careful documentation to avoid confusion.

## Hardware Integration Assessment

The hardware design is pragmatic rather than over-abstracted. The repository provides both mock and serial-backed hardware interfaces and adds a separate pendant communication channel. This is a practical pattern for robotics development because it allows development work to continue when hardware is absent.

The serial handling strategy is conservative and simple: ASCII or CSV-like payloads, newline framing, port scanning, and fallback behavior. That is appropriate for a prototype or small production system, but it leaves room for stronger protocol guarantees later.

## Visualization Assessment

The 3D visualizer is a major asset in this project. It improves debugging, operator confidence, and path verification. The recent work around weld-aware coloring makes the visualization more semantically informative, not just decorative.

The visualizer is currently embedded in the UI layer and is tightly connected to runtime data structures. That is reasonable for now, although future extraction into a dedicated widget module would reduce maintenance cost.

## Overall Conclusion

The project is a serious robotics application with meaningful architecture, not just a demo GUI. Its strongest qualities are the deterministic control-loop design, the explicit planning layer, the practical program and trajectory model, and the operator-focused UI features.

The most important improvement areas are documentation accuracy, reduction of UI-file sprawl, cleanup of backup or legacy code paths, and further hardening of hardware communication boundaries.

In its current form, the repository is usable, capable, and technically coherent, but it is still carrying the operational cost of an ongoing refactor.
