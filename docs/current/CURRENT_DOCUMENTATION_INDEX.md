# Current Documentation Index

Snapshot date: 2026-05-17

This documentation set describes the current state of the Kinmatech robotics project based on the code in the repository, not only the historical markdown files already present in the root.

## Purpose

The repository already contains several architecture and implementation markdown files. Some of those documents are historical and describe earlier refactor phases. This index points to a fresh set of current-state documents intended to answer four questions:

1. What does the project do today?
2. How is the codebase organized today?
3. How does data and control move through the system today?
4. What are the main risks, gaps, and recommended next steps?

## Documents

- `CURRENT_PROJECT_ANALYSIS.md`
  - High-level system overview
  - Primary use cases
  - Technology stack
  - Directory ownership
  - Strengths and limitations

- `CURRENT_SUBSYSTEM_REFERENCE.md`
  - Module-by-module breakdown of UI, control, planning, and hardware layers
  - Key classes and responsibilities
  - Important entrypoints and supporting scripts

- `CURRENT_RUNTIME_AND_DATAFLOW.md`
  - Startup path
  - Thread model
  - Command flow
  - Path generation flow
  - Playback and visualization flow
  - Pendant and hardware communication flow

- `CURRENT_TECHNICAL_DEBT.md`
  - Architectural drift
  - Documentation drift
  - Legacy and backup code impact
  - Priority recommendations for cleanup and hardening

## How To Use This Set

- Read `CURRENT_PROJECT_ANALYSIS.md` first if you want an executive-level understanding.
- Read `CURRENT_SUBSYSTEM_REFERENCE.md` if you need to find code quickly.
- Read `CURRENT_RUNTIME_AND_DATAFLOW.md` if you are changing motion execution, visualization, jogging, or hardware integration.
- Read `CURRENT_TECHNICAL_DEBT.md` before planning a larger refactor or production hardening effort.

## Relationship To Existing Markdown

The existing root markdown files are still useful, especially for historical implementation context. The newer `CURRENT_*` files are intended to serve as the most accurate operational description of the codebase as of the snapshot date above.
