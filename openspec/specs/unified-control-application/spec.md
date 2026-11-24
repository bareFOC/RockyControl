# unified-control-application Specification

## Purpose

Guarantees that firmware and simulation execute the same control application code path, with only I/O backends swapped, so that replay verification covers the exact code that runs on hardware and simulation results predict hardware behavior.

## Requirements

### Requirement: Single kernel wrapper
The system SHALL provide exactly one wrapper around the real-time control kernel, hosted in the shared control-application crate, and both firmware and simulation SHALL call that same wrapper to execute a control tick.

#### Scenario: Firmware and simulation use the same wrapper
- **WHEN** a control tick is executed in firmware and in simulation
- **THEN** both paths call the same `ControlTask` implementation from the shared control-application crate, with no parallel wrapper in the simulation crate

#### Scenario: No duplicate bus trait
- **WHEN** the simulation implements its servo I/O backend
- **THEN** it implements the same `ServoBusPort` trait used by the firmware rather than a separate equivalent trait

### Requirement: Deterministic host-callable control step
The control wrapper's step function SHALL be pure with respect to global state: all inputs (commands, feedback, wire error counts, time) SHALL be explicit parameters, so it produces identical results for identical inputs on host and MCU.

#### Scenario: Same inputs produce same outputs on host and MCU
- **WHEN** identical command, feedback, wire-error, and time inputs are fed to the control wrapper on the host and on the MCU build
- **THEN** both produce bit-identical telemetry and servo commands

### Requirement: Replay covers the firmware path
Deterministic replay recording and verification SHALL live in the shared control-application crate and SHALL operate on the same control wrapper that firmware uses, rather than on a simulation-only wrapper.

#### Scenario: Replay verifies firmware wrapper
- **WHEN** a recorded tick log is replayed for verification
- **THEN** verification re-executes the firmware's `ControlTask` and reports an exact match only if its telemetry and commands match the recording bit-for-bit

#### Scenario: Firmware-only wrapper bug is caught
- **WHEN** a change is made to the shared control wrapper's behavior
- **THEN** replay verification detects the behavioral difference, ensuring simulation cannot silently diverge from firmware

### Requirement: Shared transport and model store
The simulation SHALL use the shared host-link transport and flash-model-store implementation rather than parallel reimplementations.

#### Scenario: Simulation uses shared COBS transport
- **WHEN** the simulation exchanges host/MCU frames
- **THEN** it uses the shared control-application crate's COBS framing and mailbox types

#### Scenario: Simulation uses shared model store
- **WHEN** the simulation handles a model-info or model-chunk request
- **THEN** it uses the shared `FlashModelStore` implementation
