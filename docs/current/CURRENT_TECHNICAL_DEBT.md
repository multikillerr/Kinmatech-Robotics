# Current Technical Debt And Recommendations

Snapshot date: 2026-05-17

## Overview

The repository is functional and well beyond prototype level, but it carries technical debt from rapid feature growth and an in-progress architectural migration. The debt is manageable, but it is now material enough that future development speed and correctness will benefit from deliberate cleanup.

## Major Debt Categories

### 1. Documentation drift

Risk:

Several existing markdown files describe earlier states of the project. This makes onboarding and change planning slower because the reader must first determine which document is historical and which is current.

Impact:

- higher onboarding cost
- incorrect mental models
- risk of implementing against obsolete architecture assumptions

Recommendation:

- treat the `CURRENT_*` files as the authoritative current-state set
- mark older root markdown files as historical or snapshot-specific where appropriate
- update the top-level README to point at the current-state documentation index

### 2. UI concentration risk

Risk:

`ui/main.py` is very large and owns too many responsibilities:

- window construction
- user actions
- visualization
- thread startup
- runtime coordination
- pendant handling
- logging behavior
- program editing behavior

Impact:

- high merge-conflict probability
- difficult localized testing
- higher regression risk for unrelated edits
- slower navigation for contributors

Recommendation:

Break the file down incrementally, not through a large rewrite. Good extraction candidates are:

- visualizer widget module
- program-table controller
- pendant integration adapter
- toolbar or mode controller
- file IO coordinator

### 3. Legacy and backup code paths

Risk:

The repository contains `backup` directories and root-level historical files with overlapping names and responsibilities.

Impact:

- uncertainty about the canonical implementation
- accidental edits in stale files
- duplicated bug fixes
- harder code search results

Recommendation:

- clearly mark backup directories as non-runtime
- move them under a dedicated archive area if they must remain
- document canonical file paths for all active subsystems

### 4. Architectural purity gaps

Risk:

The codebase is moving toward a four-layer architecture, but some cross-layer leakage and compatibility imports remain.

Impact:

- future refactors will be more expensive than they need to be
- layer ownership can become ambiguous
- harder to test modules in isolation

Recommendation:

- define a canonical import policy for new code
- prohibit new root-level compatibility imports unless required
- prefer `planning.`, `control.`, `hardware.`, and `ui.` module imports explicitly

### 5. Serial protocol simplicity

Risk:

The robot interface uses a simple ASCII CSV protocol. That is easy to debug but relatively weak in terms of framing guarantees, validation, and versioning.

Impact:

- poor resilience to malformed input
- limited extensibility if message types grow
- less robust debugging when streams become more complex

Recommendation:

Near term:

- formalize command and feedback line formats in one document
- validate field counts and ranges more aggressively

Longer term:

- consider explicit message prefixes or message types
- add checksums or stronger framing if traffic complexity increases

### 6. Overlapping hardware pathways

Risk:

The repository includes multiple hardware-related mechanisms: hardware thread, feedback daemons, enhanced feedback daemon, pendant thread, and supporting command modules.

Impact:

- duplication of responsibility
- unclear source of truth for feedback
- future synchronization issues if multiple pathways stay active

Recommendation:

- document which hardware path is canonical for current runtime
- deprecate or isolate non-canonical paths
- ensure a single authoritative source exists for current joint feedback during execution

## Strategic Recommendations

## Priority 1: Documentation and source-of-truth cleanup

Do this first because it lowers the cost of every later change.

Actions:

- link the current-state docs from the README
- mark historical docs as historical
- define canonical runtime file paths

## Priority 2: `ui/main.py` decomposition

Do this second because it is the largest ongoing maintainability risk.

Recommended extraction order:

1. visualizer module
2. pendant command adapter
3. program-table controller
4. path playback controller

## Priority 3: Hardware boundary hardening

Do this before scaling hardware features.

Actions:

- formalize robot serial protocol
- formalize pendant protocol
- define port-selection strategy and failure behavior
- centralize reconnect policy and logging rules

## Priority 4: Legacy pruning

Do this once the canonical paths are fully agreed.

Actions:

- move backup directories into a clearly named archival area
- add a short archive readme if retention is required
- remove or quarantine obsolete root-level duplicates

## Recommended Future Documentation Additions

The following documents would be valuable later, but they are secondary to the current-state set already created:

- protocol specification for robot and pendant serial messages
- operator manual for jogging, teaching, path generation, and playback
- deployment guide for real hardware bring-up
- troubleshooting guide keyed by symptoms and log messages
- test strategy document for simulation versus hardware environments

## Final Assessment

The project’s debt is not a sign of poor engineering. It is the predictable result of a real robotics application growing through active experimentation, feature delivery, and architectural transition.

The strongest next move is not a rewrite. It is disciplined consolidation:

- cleaner current documentation,
- clearer canonical module ownership,
- smaller UI surfaces,
- and more explicit hardware contracts.

If that consolidation is done well, the repository will become much easier to scale without losing the practical engineering gains already present.
