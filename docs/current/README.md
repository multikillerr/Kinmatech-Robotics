# Current Kinmatech Robotics Documentation

Snapshot date: **18 May 2026**

This directory contains the authoritative current-state documentation for the Kinmatech Robotics project. These documents describe the codebase, architecture, and runtime behavior **as it exists today**, not historical or aspirational states.

## Quick Navigation

**Start here if you want to...**

- **Understand what the project does**: [CURRENT_PROJECT_ANALYSIS.md](CURRENT_PROJECT_ANALYSIS.md)
  - Executive summary, technology stack, strengths and weaknesses, what the system does

- **Find a specific code module**: [CURRENT_SUBSYSTEM_REFERENCE.md](CURRENT_SUBSYSTEM_REFERENCE.md)
  - Module-by-module breakdown of all layers (UI, control, planning, hardware)
  - Key classes, responsibilities, and file locations

- **Understand how data flows**: [CURRENT_RUNTIME_AND_DATAFLOW.md](CURRENT_RUNTIME_AND_DATAFLOW.md)
  - Startup sequence, thread model, command flow, path generation, visualization, hardware communication
  - Read this before modifying motion execution or jogging behavior

- **Plan a larger refactor**: [CURRENT_TECHNICAL_DEBT.md](CURRENT_TECHNICAL_DEBT.md)
  - Known architectural debt, documentation drift, legacy code, and recommendations for cleanup
  - Priority roadmap for hardening and consolidation

- **See the full documentation index**: [CURRENT_DOCUMENTATION_INDEX.md](CURRENT_DOCUMENTATION_INDEX.md)
  - Relationship to older markdown files and guidance on when to use each document

## Document Overview

### [CURRENT_PROJECT_ANALYSIS.md](CURRENT_PROJECT_ANALYSIS.md)

**For**: Getting oriented to the project at a high level.

Covers:
- What the system does (operator use cases)
- Technology stack (PyQt6, kinematics, OpenGL, serial)
- Architectural shape (4-layer model)
- Primary entry points
- Strengths and weaknesses
- Overall conclusion about project maturity and next steps

Read this first if you are new to the codebase.

### [CURRENT_SUBSYSTEM_REFERENCE.md](CURRENT_SUBSYSTEM_REFERENCE.md)

**For**: Quickly finding where code lives and who owns what.

Covers:
- Top-level entrypoints (launcher.py, ui/main.py)
- UI layer (main.py, program_table_model.py, visualizer)
- Control layer (threads.py, command queue, state machine)
- Planning layer (kinematics, path generation, data models)
- Hardware layer (robot interface, pendant, feedback)
- Data and content directories (positions/, logs/)
- Subsystem ownership guide

Use this as a quick lookup when you need to find code or understand module responsibility.

### [CURRENT_RUNTIME_AND_DATAFLOW.md](CURRENT_RUNTIME_AND_DATAFLOW.md)

**For**: Understanding how the system executes and moves data.

Covers:
- Runtime overview and thread model
- Startup flow
- Command flow (jog, program authoring, path generation, playback)
- Visualization flow
- File formats (JSON programs, KSM trajectories, serial protocols)
- Port detection and pendant handshake
- Operational boundaries

Read this before changing motion execution, adding new command types, or modifying the pendant or hardware interface.

### [CURRENT_TECHNICAL_DEBT.md](CURRENT_TECHNICAL_DEBT.md)

**For**: Planning refactoring work or understanding maintenance costs.

Covers:
- Documentation drift
- UI concentration risk
- Legacy and backup code paths
- Architectural purity gaps
- Serial protocol simplicity
- Overlapping hardware pathways
- Strategic recommendations (priority 1-4)
- Future documentation needs

Read this before proposing large structural changes or before production hardening.

### [CURRENT_DOCUMENTATION_INDEX.md](CURRENT_DOCUMENTATION_INDEX.md)

**For**: Understanding the documentation landscape as a whole.

Covers:
- Purpose of this documentation set
- What distinguishes current docs from older markdown files
- How to use each document
- Relationship to existing markdown

Useful for context about why these documents exist separately from older files.

## Key Facts

- **Project type**: Desktop robotics control application
- **Main language**: Python 3 with PyQt6
- **Architecture**: 4-layer (UI, control, planning, hardware)
- **Control loop**: 100 Hz fixed-rate (10 ms cycle)
- **Visualization**: OpenGL-based 3D viewer with semantic coloring
- **Operator surface**: Jog, teach, program, generate, execute, visualize
- **Hardware**: Serial communication with robot and pendant

## Relationship To Other Documentation

The repository root contains older markdown files (ARCHITECTURE.md, IMPLEMENTATION_SUMMARY.md, CONTROL_LOOP_*, etc.). Those documents are historically useful but may describe earlier refactor phases.

**Use the current documents as authoritative for:**
- Current runtime behavior
- Where to find code
- How to change motion execution
- Planning refactoring work

**Use the older documents for:**
- Historical context about design decisions
- Earlier implementation phases
- Migration artifacts

## Contributing

When adding new code or changing behavior:

1. Check [CURRENT_SUBSYSTEM_REFERENCE.md](CURRENT_SUBSYSTEM_REFERENCE.md) to confirm module ownership.
2. Read [CURRENT_RUNTIME_AND_DATAFLOW.md](CURRENT_RUNTIME_AND_DATAFLOW.md) if your change affects motion, visualization, or hardware.
3. If you plan a larger refactor, read [CURRENT_TECHNICAL_DEBT.md](CURRENT_TECHNICAL_DEBT.md) for priority guidance.
4. Keep these docs in sync as you change behavior. Stale documentation is worse than no documentation.

## Questions?

- **"Where is the robot control loop?"** → [CURRENT_SUBSYSTEM_REFERENCE.md](CURRENT_SUBSYSTEM_REFERENCE.md#control-layer), then [CURRENT_RUNTIME_AND_DATAFLOW.md](CURRENT_RUNTIME_AND_DATAFLOW.md#thread-model)
- **"How do I add a new jog command?"** → [CURRENT_RUNTIME_AND_DATAFLOW.md](CURRENT_RUNTIME_AND_DATAFLOW.md#command-flow)
- **"What's the serial protocol?"** → [CURRENT_RUNTIME_AND_DATAFLOW.md](CURRENT_RUNTIME_AND_DATAFLOW.md#file-formats-and-data-movement)
- **"Should I refactor the UI?"** → [CURRENT_TECHNICAL_DEBT.md](CURRENT_TECHNICAL_DEBT.md#priority-2-uimainpy-decomposition)
- **"What are the backup directories?"** → [CURRENT_TECHNICAL_DEBT.md](CURRENT_TECHNICAL_DEBT.md#3-legacy-and-backup-code-paths)
