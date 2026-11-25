# sim-verification-gate Specification

## Purpose

An automated verification gate and committed evidence define and prove that the software simulation is complete before hardware work begins.

## Requirements

### Requirement: Automated simulation verification gate
The repository SHALL provide an xtask command and a CI workflow that verify host crates, firmware-core, the RP2350 feature matrix, WASM and the simulation acceptance suite. The gate SHALL fail on any test failure and SHALL run committed scripted experiments with replay verification.

#### Scenario: Gate passes on a clean tree
- **WHEN** the sim-verify gate runs on the committed tree
- **THEN** all host tests, firmware tests, cross-target checks, scripted experiments and replay verifications pass

#### Scenario: Gate uses current crate names
- **WHEN** the gate is invoked
- **THEN** it references only `rk-*` crates and paths that exist in the workspace

### Requirement: Committed simulation evidence
Golden scripted experiments and at least one golden replay log SHALL be committed and verified by the gate, and a single acceptance document SHALL record behavior, test, and measured result.

#### Scenario: Golden scripts are reproducible
- **WHEN** CI runs the committed experiment scripts
- **THEN** each reports `exact_match` with the recorded tick count and meets its quality bounds

#### Scenario: Acceptance scope is explicit
- **WHEN** a developer reads the acceptance document
- **THEN** it states that replay verifies the kernel tick stream and not host frame pacing

### Requirement: Consistent fault defaults and clean reset
Simulator fault defaults SHALL be identical across the simulator and replay snapshot types, and resetting an experiment SHALL clear fall alarm, recovery state and operator consent.

#### Scenario: Fault defaults match
- **WHEN** a `SimFaultInjection` default is converted to a `SimFaultSnapshot`
- **THEN** the two are equal field-by-field

#### Scenario: Reset clears fall state
- **WHEN** a fall alarm is latched and the experiment is reset
- **THEN** no stale alarm, recovery state or consent remains and a subsequent shove behaves as in a fresh session

### Requirement: Simulated per-joint bus degradation
The simulator SHALL support one or more individual servo joints degrading while other joints keep replying, using the same bus-health tracker as firmware.

#### Scenario: Single joint degraded
- **WHEN** one joint's register transactions fail repeatedly
- **THEN** only that joint is marked degraded, the timeout count increments for it, and all other joints continue to provide feedback

### Requirement: Tool and feature end-to-end coverage
MCP tool behavior, the live GUI HTTP listener, running/CBF/impact-through-app, side-orientation recovery, and staged MCU model pull SHALL have end-to-end tests in simulation.

#### Scenario: MCP tools drive the simulator
- **WHEN** an MCP client calls action playback, gait, telemetry, calibration, URDF/model and estop-reset tools
- **THEN** each returns a valid result and the simulator state changes accordingly

#### Scenario: GUI listener drives a sim tool
- **WHEN** an HTTP client calls `run_experiment` through the desktop GUI listener
- **THEN** the GUI executes it against the simulation and returns a replay verification

#### Scenario: Staged model pull
- **WHEN** a model is staged on the simulated MCU and pulled through the chunked protocol with an injected CRC failure
- **THEN** retry succeeds and the pulled model matches the source
