# Spec Delta

## MODIFIED Requirements

### Requirement: Single kernel wrapper
The system SHALL provide exactly one wrapper around the real-time control kernel, hosted in the shared control-application crate, and both firmware and simulation SHALL call that same wrapper to execute a control tick. The wrapper SHALL be driven by one Embassy control task through a shared I/O trait.

#### Scenario: Firmware and simulation use the same wrapper
- **WHEN** a control tick is executed in firmware and in simulation
- **THEN** both paths call the same kernel wrapper from a single Embassy task, with only the I/O backend differing

#### Scenario: No duplicate bus trait
- **WHEN** the simulation implements its servo I/O backend
- **THEN** it implements the same `ServoBusPort`/`RobotIo` contract used by the firmware rather than a separate equivalent trait

#### Scenario: No duplicate task code
- **WHEN** the simulation is built for WASM
- **THEN** it does not contain a parallel copy of the control loop or scheduler
