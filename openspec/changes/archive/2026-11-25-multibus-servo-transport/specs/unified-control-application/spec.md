# Spec Delta

## MODIFIED Requirements

### Requirement: Single kernel wrapper
The system SHALL provide exactly one wrapper around the real-time control kernel, hosted in the shared control-application crate, and both firmware and simulation SHALL call that same wrapper to execute a control tick. All simulation callers, including MCP hosts, SHALL use the deterministic fixed-step transport path. The servo bus beneath the wrapper SHALL use BareFOC register-pointer transactions via the shared `rk-multibus` client; no separate bespoke per-joint frame codec SHALL remain.

#### Scenario: Firmware and simulation use the same wrapper
- **WHEN** a control tick is executed in firmware and in simulation
- **THEN** both paths call the same `ControlTask` implementation from the shared control-application crate, with no parallel wrapper in the simulation crate

#### Scenario: No duplicate bus trait
- **WHEN** the simulation implements its servo I/O backend
- **THEN** it implements the same `ServoBusPort`/`RobotIo` contract used by the firmware rather than a separate equivalent trait

#### Scenario: No duplicate task code
- **WHEN** the simulation is built for WASM
- **THEN** it does not contain a parallel copy of the control loop or scheduler

#### Scenario: No legacy frame codec
- **WHEN** the servo bus payload is inspected
- **THEN** the former 16-byte command / 12-byte feedback pack/unpack functions are absent
